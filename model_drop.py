"""Drag-and-drop support for loading one STEP/IGES model into the OCC viewer.

The module deliberately does not import Qt or pythonOCC at import time.  Path
extraction and validation are therefore unit-testable in the lightweight test
environment, while :func:`install_model_drop_support` receives the active Qt
module from ``main.py``.

The application currently owns one active CAD model.  A directory drop is
accepted only when its top level contains exactly one supported model, and a
multi-file drop is rejected when it resolves to more than one model.  This
avoids silently replacing the first dropped model with the last one.
"""

from dataclasses import dataclass
import os
import re
from typing import Any, Callable, Iterable, List, Optional, Tuple
from urllib.parse import unquote, urlsplit


SUPPORTED_MODEL_EXTENSIONS = frozenset((".step", ".stp", ".iges", ".igs"))
DEFAULT_DIRECTORY_SCAN_LIMIT = 5000

# English defaults keep this module independent from the application's i18n
# singleton.  The Qt filter can receive ``i18n.tr`` (or another compatible
# callable), while lightweight callers still get useful English feedback.
_ENGLISH_DROP_TEXT = {
    "drop.issue.local_only":
        "Only local files can be imported (received {scheme} URI)",
    "drop.issue.no_path": "The dropped item has no path",
    "drop.issue.invalid_utf8": "The dropped path is not valid UTF-8",
    "drop.issue.empty_path": "The dropped item has an empty path",
    "drop.issue.empty_local_path": "The dropped item has an empty local path",
    "drop.issue.directory_limit":
        "Directory contains more than {limit} entries; drop one model file directly",
    "drop.issue.directory_read": "Cannot read directory: {error}",
    "drop.issue.directory_no_model":
        "Directory has no STEP/IGES model in its top level",
    "drop.issue.unsupported_format":
        "Unsupported format {extension} for {filename}; accepted extensions are "
        ".step, .stp, .iges, .igs",
    "drop.issue.not_regular":
        "Dropped item is not a regular file or directory: {filename}",
    "drop.issue.not_found": "File or directory does not exist: {filename}",
    "drop.feedback.release": "Release to import model: {filename}",
    "drop.feedback.multiple":
        "{count} supported models detected; this viewer accepts one model at a time",
    "drop.feedback.prompt": "Drop one STEP or IGES model file",
    "drop.error.multiple":
        "The drop contains {count} supported models, but the application owns one "
        "active model at a time. Drop a single file.",
    "drop.error.issue_header": "The dropped item cannot be imported:",
    "drop.error.issue_line": "- {source}: {reason}",
    "drop.error.path_line": "- {path}",
    "drop.error.prompt": "Drop one local STEP or IGES model file.",
    "drop.error.more": "... and {count} more",
    "drop.status.loading": "Loading dropped model: {filename}",
    "drop.status.start_failed": "Could not start model import: {error}",
}

_WINDOWS_DRIVE_PATH = re.compile(r"^[A-Za-z]:[\\/]")
_WINDOWS_URI_PATH = re.compile(r"^/[A-Za-z]:[\\/]")


@dataclass(frozen=True)
class DropText:
    """Semantic drag/drop message whose values survive language switches."""

    key: str
    values: Tuple[Tuple[str, Any], ...] = ()

    def render(self, translate: Optional[Callable[..., str]] = None) -> str:
        default = _ENGLISH_DROP_TEXT.get(self.key, self.key)
        values = dict(self.values)
        if translate is not None:
            try:
                rendered = translate(self.key, default=default, **values)
                if isinstance(rendered, str) and rendered != self.key:
                    return rendered
            except TypeError:
                # Also accept a lightweight ``translate(key, **values)``
                # callable; the application-level i18n service supports the
                # richer ``default=`` contract directly.
                try:
                    rendered = translate(self.key, **values)
                    if isinstance(rendered, str) and rendered != self.key:
                        return rendered
                except Exception:
                    pass
            except Exception:
                # Drag events must remain usable if a custom translator fails.
                pass
        try:
            return default.format(**values)
        except (KeyError, IndexError, ValueError, AttributeError):
            return default

    def __str__(self) -> str:
        return self.render()


def _drop_text(key: str, **values: Any) -> DropText:
    return DropText(key, tuple(sorted(values.items())))


@dataclass(frozen=True)
class DropIssue:
    """One source that could not safely resolve to an importable CAD file."""

    source: str
    message: DropText

    @property
    def reason(self) -> str:
        """Backward-compatible English representation for non-UI callers."""
        return self.message.render()

    @property
    def reason_key(self) -> str:
        return self.message.key

    @property
    def reason_values(self) -> dict:
        return dict(self.message.values)


@dataclass(frozen=True)
class ModelDropResolution:
    """Validated result of a drag payload."""

    model_paths: Tuple[str, ...]
    issues: Tuple[DropIssue, ...]

    @property
    def is_importable(self) -> bool:
        """True only when the payload unambiguously selects one model."""
        # Reject mixed valid/invalid direct drops instead of silently ignoring
        # a companion item the user reasonably expected us to import.
        return len(self.model_paths) == 1 and not self.issues

    @property
    def import_path(self) -> Optional[str]:
        return self.model_paths[0] if self.is_importable else None


def is_supported_model_path(path: str) -> bool:
    """Return whether *path* has a supported CAD extension."""
    return os.path.splitext(os.fspath(path))[1].lower() in SUPPORTED_MODEL_EXTENSIONS


def _strip_matching_quotes(text: str) -> str:
    text = text.strip().strip("\x00")
    if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
        return text[1:-1].strip()
    return text


def _file_uri_to_path(text: str) -> Tuple[Optional[str], Optional[DropText]]:
    """Translate a file URI while preserving Windows drive and UNC paths."""
    parsed = urlsplit(text)
    scheme = parsed.scheme.lower()
    if scheme and scheme != "file":
        return None, _drop_text("drop.issue.local_only", scheme=parsed.scheme)
    if scheme != "file":
        return text, None

    netloc = unquote(parsed.netloc)
    path = unquote(parsed.path)
    if len(netloc) == 2 and netloc[1] == ":":
        # Tolerate file://D:/folder/model.step in addition to file:///D:/...
        path = netloc + path
    elif netloc and netloc.lower() != "localhost":
        path = "//{}{}".format(netloc, path)
    elif _WINDOWS_URI_PATH.match(path):
        path = path[1:]
    return path, None


def local_path_from_drop_source(
        source: Any) -> Tuple[Optional[str], Optional[DropText]]:
    """Resolve a Qt ``QUrl``-like object, URI, or plain path to a local path.

    The duck-typed QUrl support keeps this module independent of PyQt/PySide.
    """
    if source is None:
        return None, _drop_text("drop.issue.no_path")

    if hasattr(source, "toLocalFile"):
        try:
            local = source.toLocalFile()
        except Exception:
            local = ""
        if local:
            text = os.fspath(local)
        else:
            try:
                text = source.toString()
            except Exception:
                text = str(source)
    elif isinstance(source, bytes):
        try:
            text = source.decode("utf-8")
        except UnicodeDecodeError:
            return None, _drop_text("drop.issue.invalid_utf8")
    else:
        try:
            text = os.fspath(source)
        except TypeError:
            text = str(source)

    text = _strip_matching_quotes(str(text))
    if not text:
        return None, _drop_text("drop.issue.empty_path")

    # urlsplit treats ``D:\\folder`` as URI scheme ``d``; recognise Windows
    # paths before parsing a possible URI.
    if _WINDOWS_DRIVE_PATH.match(text) or text.startswith("\\\\"):
        path, error = text, None
    else:
        path, error = _file_uri_to_path(text)
    if error:
        return None, error
    if not path:
        return None, _drop_text("drop.issue.empty_local_path")

    path = os.path.expanduser(path)
    return os.path.normpath(os.path.abspath(path)), None


def sources_from_mime_data(mime_data: Any) -> Tuple[Any, ...]:
    """Extract QUrls or newline-delimited plain paths from ``QMimeData``."""
    if mime_data is None:
        return ()

    try:
        if mime_data.hasUrls():
            urls = tuple(mime_data.urls())
            if urls:
                return urls
    except Exception:
        pass

    try:
        if not mime_data.hasText():
            return ()
        text = str(mime_data.text())
    except Exception:
        return ()

    # text/uri-list permits comment lines.  Do not split on spaces because
    # ordinary Windows paths frequently contain them.
    return tuple(line.strip() for line in text.splitlines()
                 if line.strip() and not line.lstrip().startswith("#"))


def _scan_directory(directory: str, scan_limit: int) -> Tuple[List[str], Optional[DropIssue]]:
    models: List[str] = []
    scanned = 0
    try:
        with os.scandir(directory) as entries:
            for entry in entries:
                scanned += 1
                if scanned > scan_limit:
                    return [], DropIssue(
                        directory,
                        _drop_text("drop.issue.directory_limit",
                                   limit=scan_limit))
                try:
                    is_file = entry.is_file(follow_symlinks=True)
                except OSError:
                    is_file = False
                if is_file and is_supported_model_path(entry.name):
                    models.append(os.path.abspath(entry.path))
    except OSError as exc:
        return [], DropIssue(
            directory,
            _drop_text("drop.issue.directory_read", error=str(exc)))

    if not models:
        return [], DropIssue(
            directory,
            _drop_text("drop.issue.directory_no_model"))
    models.sort(key=lambda item: os.path.normcase(item))
    return models, None


def resolve_model_drop(
        sources: Iterable[Any],
        directory_scan_limit: int = DEFAULT_DIRECTORY_SCAN_LIMIT,
) -> ModelDropResolution:
    """Resolve drag sources to local supported files without importing them.

    Directories are scanned non-recursively.  The result is importable only
    when exactly one unique STEP/IGES file is found.
    """
    if directory_scan_limit < 1:
        raise ValueError("directory_scan_limit must be at least 1")

    models: List[str] = []
    issues: List[DropIssue] = []
    seen = set()

    for source in sources:
        source_label = str(source)
        path, error = local_path_from_drop_source(source)
        if error:
            issues.append(DropIssue(source_label, error))
            continue
        assert path is not None

        if os.path.isdir(path):
            directory_models, issue = _scan_directory(path, directory_scan_limit)
            if issue:
                issues.append(issue)
            candidates = directory_models
        elif os.path.isfile(path):
            if not is_supported_model_path(path):
                extension = os.path.splitext(path)[1].lower() or "(none)"
                issues.append(DropIssue(
                    path,
                    _drop_text(
                        "drop.issue.unsupported_format",
                        extension=extension,
                        filename=os.path.basename(path))))
                continue
            candidates = [path]
        elif os.path.exists(path):
            issues.append(DropIssue(
                path,
                _drop_text("drop.issue.not_regular",
                           filename=os.path.basename(path) or path)))
            continue
        else:
            issues.append(DropIssue(
                path,
                _drop_text("drop.issue.not_found",
                           filename=os.path.basename(path) or path)))
            continue

        for candidate in candidates:
            key = os.path.normcase(os.path.abspath(candidate))
            if key not in seen:
                seen.add(key)
                models.append(os.path.abspath(candidate))

    models.sort(key=lambda item: os.path.normcase(item))
    return ModelDropResolution(tuple(models), tuple(issues))


def resolve_mime_model_drop(mime_data: Any) -> ModelDropResolution:
    return resolve_model_drop(sources_from_mime_data(mime_data))


def format_drop_feedback(
        resolution: ModelDropResolution,
        translate: Optional[Callable[..., str]] = None,
) -> str:
    """Render short status feedback in the language active at call time."""
    if resolution.is_importable:
        return _drop_text(
            "drop.feedback.release",
            filename=os.path.basename(resolution.import_path or ""),
        ).render(translate)
    if len(resolution.model_paths) > 1:
        return _drop_text(
            "drop.feedback.multiple",
            count=len(resolution.model_paths),
        ).render(translate)
    if resolution.issues:
        return resolution.issues[0].message.render(translate)
    return _drop_text("drop.feedback.prompt").render(translate)


def format_drop_error(
        resolution: ModelDropResolution,
        max_items: int = 4,
        translate: Optional[Callable[..., str]] = None,
) -> str:
    """Render detailed rejection text in the language active at call time."""
    if len(resolution.model_paths) > 1:
        lines = [
            _drop_text(
                "drop.error.multiple",
                count=len(resolution.model_paths),
            ).render(translate)
        ]
        lines.extend(
            _drop_text("drop.error.path_line", path=path).render(translate)
            for path in resolution.model_paths[:max_items])
    elif resolution.issues:
        lines = [_drop_text("drop.error.issue_header").render(translate)]
        lines.extend(
            _drop_text(
                "drop.error.issue_line",
                source=issue.source,
                reason=issue.message.render(translate),
            ).render(translate)
            for issue in resolution.issues[:max_items])
    else:
        lines = [_drop_text("drop.error.prompt").render(translate)]

    omitted = max(0, len(resolution.model_paths) - max_items)
    if not resolution.model_paths:
        omitted = max(0, len(resolution.issues) - max_items)
    if omitted:
        lines.append(_drop_text(
            "drop.error.more", count=omitted).render(translate))
    return "\n".join(lines)


def _qevent_value(QtCore: Any, name: str) -> Any:
    """Return a QEvent value across Qt5 and Qt6 enum layouts."""
    direct = getattr(QtCore.QEvent, name, None)
    if direct is not None:
        return direct
    event_type = getattr(QtCore.QEvent, "Type", None)
    return getattr(event_type, name, None) if event_type is not None else None


def create_model_drop_filter_class(QtCore: Any):
    """Create a Qt-binding-specific event filter class lazily."""
    drag_enter = _qevent_value(QtCore, "DragEnter")
    drag_move = _qevent_value(QtCore, "DragMove")
    drag_leave = _qevent_value(QtCore, "DragLeave")
    drop = _qevent_value(QtCore, "Drop")

    class ModelDropEventFilter(QtCore.QObject):
        def __init__(self, viewer_widget: Any,
                     on_model_path: Callable[[str], Any],
                     show_error: Optional[Callable[[str], Any]] = None,
                     set_status: Optional[Callable[[str], Any]] = None,
                     translate: Optional[Callable[..., str]] = None):
            super(ModelDropEventFilter, self).__init__(viewer_widget)
            self.viewer_widget = viewer_widget
            self.on_model_path = on_model_path
            self.show_error = show_error
            self.set_status = set_status
            self.translate = translate
            self._cached_mime_id = None
            self._cached_resolution = None
            self._last_status = None

        def _status(self, message: str) -> None:
            if message == self._last_status:
                return
            self._last_status = message
            if self.set_status:
                self.set_status(message)

        def _resolution(self, event: Any) -> ModelDropResolution:
            mime_data = event.mimeData()
            mime_id = id(mime_data)
            if mime_id != self._cached_mime_id:
                self._cached_mime_id = mime_id
                self._cached_resolution = resolve_mime_model_drop(mime_data)
            return self._cached_resolution

        def _clear_cache(self) -> None:
            self._cached_mime_id = None
            self._cached_resolution = None

        def eventFilter(self, watched: Any, event: Any) -> bool:
            event_type = event.type()
            if event_type in (drag_enter, drag_move):
                resolution = self._resolution(event)
                self._status(format_drop_feedback(
                    resolution, translate=self.translate))
                if resolution.is_importable:
                    event.acceptProposedAction()
                else:
                    event.ignore()
                return True

            if event_type == drag_leave:
                self._clear_cache()
                self._status("")
                if hasattr(event, "accept"):
                    event.accept()
                return True

            if event_type == drop:
                resolution = self._resolution(event)
                self._clear_cache()
                if not resolution.is_importable:
                    event.ignore()
                    self._status(format_drop_feedback(
                        resolution, translate=self.translate))
                    if self.show_error:
                        self.show_error(format_drop_error(
                            resolution, translate=self.translate))
                    return True

                path = resolution.import_path
                assert path is not None
                event.acceptProposedAction()
                self._status(_drop_text(
                    "drop.status.loading",
                    filename=os.path.basename(path),
                ).render(self.translate))
                try:
                    self.on_model_path(path)
                except Exception as exc:
                    message = _drop_text(
                        "drop.status.start_failed", error=str(exc),
                    ).render(self.translate)
                    self._status(message)
                    if self.show_error:
                        self.show_error(message)
                return True

            return super(ModelDropEventFilter, self).eventFilter(watched, event)

    return ModelDropEventFilter


def install_model_drop_support(
        QtCore: Any,
        viewer_widget: Any,
        on_model_path: Callable[[str], Any],
        show_error: Optional[Callable[[str], Any]] = None,
        set_status: Optional[Callable[[str], Any]] = None,
        translate: Optional[Callable[..., str]] = None,
) -> Any:
    """Enable model drops on the pythonOCC central viewer widget.

    The returned QObject is also retained on ``viewer_widget`` so Python does
    not garbage-collect the event filter while Qt still uses it.
    """
    if viewer_widget is None:
        raise ValueError("viewer_widget is required")
    if not callable(on_model_path):
        raise TypeError("on_model_path must be callable")
    if translate is not None and not callable(translate):
        raise TypeError("translate must be callable")

    filter_class = create_model_drop_filter_class(QtCore)
    event_filter = filter_class(
        viewer_widget, on_model_path, show_error, set_status, translate)
    viewer_widget.setAcceptDrops(True)
    viewer_widget.installEventFilter(event_filter)
    viewer_widget._model_drop_event_filter = event_filter
    return event_filter
