import json
import os
from pathlib import Path
import tempfile
import unittest

from model_drop import (
    _ENGLISH_DROP_TEXT,
    format_drop_error,
    format_drop_feedback,
    install_model_drop_support,
    is_supported_model_path,
    local_path_from_drop_source,
    resolve_mime_model_drop,
    resolve_model_drop,
    sources_from_mime_data,
)
from i18n import TranslationManager


class FakeUrl:
    def __init__(self, local_path="", text=""):
        self._local_path = local_path
        self._text = text

    def toLocalFile(self):
        return self._local_path

    def toString(self):
        return self._text


class FakeMimeData:
    def __init__(self, urls=(), text=""):
        self._urls = tuple(urls)
        self._text = text

    def hasUrls(self):
        return bool(self._urls)

    def urls(self):
        return list(self._urls)

    def hasText(self):
        return bool(self._text)

    def text(self):
        return self._text


class FakeQObject:
    def __init__(self, parent=None):
        self.parent = parent

    def eventFilter(self, watched, event):
        return False


class FakeQEvent:
    DragEnter = 1
    DragMove = 2
    DragLeave = 3
    Drop = 4


class FakeQtCore:
    QObject = FakeQObject
    QEvent = FakeQEvent


class FakeViewer:
    def __init__(self):
        self.accept_drops = False
        self.event_filter = None

    def setAcceptDrops(self, value):
        self.accept_drops = value

    def installEventFilter(self, event_filter):
        self.event_filter = event_filter


class FakeDropEvent:
    def __init__(self, event_type, mime_data):
        self._event_type = event_type
        self._mime_data = mime_data
        self.accepted = False
        self.ignored = False

    def type(self):
        return self._event_type

    def mimeData(self):
        return self._mime_data

    def acceptProposedAction(self):
        self.accepted = True

    def ignore(self):
        self.ignored = True

    def accept(self):
        self.accepted = True


class ModelDropTests(unittest.TestCase):
    def test_supported_extensions_are_case_insensitive(self):
        for name in ("part.step", "part.STP", "part.iges", "part.IGS"):
            self.assertTrue(is_supported_model_path(name), name)
        self.assertFalse(is_supported_model_path("part.stl"))

    def test_all_semantic_drop_keys_exist_in_both_catalogs(self):
        locales_dir = Path(__file__).resolve().parents[1] / "locales"
        with (locales_dir / "en.json").open(encoding="utf-8") as stream:
            english = json.load(stream)["strings"]
        with (locales_dir / "zh_CN.json").open(encoding="utf-8") as stream:
            chinese = json.load(stream)["strings"]

        self.assertTrue(set(_ENGLISH_DROP_TEXT).issubset(english))
        self.assertTrue(set(_ENGLISH_DROP_TEXT).issubset(chinese))
        for key, fallback in _ENGLISH_DROP_TEXT.items():
            self.assertEqual(english[key], fallback, key)

    def test_qurl_local_file_preserves_path_with_spaces(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            model = Path(temp_dir, "model with spaces.STEP")
            model.touch()
            path, error = local_path_from_drop_source(FakeUrl(str(model)))
            self.assertIsNone(error)
            self.assertEqual(os.path.normcase(path), os.path.normcase(str(model)))

    def test_file_uri_decodes_spaces(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            model = Path(temp_dir, "model with spaces.step")
            model.touch()
            uri = model.as_uri()
            resolution = resolve_model_drop([uri])
            self.assertTrue(resolution.is_importable)
            self.assertEqual(
                os.path.normcase(resolution.import_path),
                os.path.normcase(str(model)))

    def test_non_local_uri_is_rejected(self):
        resolution = resolve_model_drop(["https://example.com/model.step"])
        self.assertFalse(resolution.is_importable)
        self.assertIn("Only local files", resolution.issues[0].reason)

    def test_directory_with_one_model_resolves_non_recursively(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            model = Path(temp_dir, "only.step")
            model.touch()
            Path(temp_dir, "notes.txt").touch()
            nested = Path(temp_dir, "nested")
            nested.mkdir()
            Path(nested, "ignored.iges").touch()

            resolution = resolve_model_drop([temp_dir])
            self.assertTrue(resolution.is_importable)
            self.assertEqual(Path(resolution.import_path).name, "only.step")

    def test_directory_with_multiple_models_is_ambiguous(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            Path(temp_dir, "a.step").touch()
            Path(temp_dir, "b.iges").touch()
            resolution = resolve_model_drop([temp_dir])
            self.assertFalse(resolution.is_importable)
            self.assertEqual(len(resolution.model_paths), 2)
            self.assertIn("one active model", format_drop_error(resolution))

    def test_multi_file_drop_deduplicates_but_rejects_two_models(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            first = Path(temp_dir, "a.step")
            second = Path(temp_dir, "b.stp")
            first.touch()
            second.touch()
            resolution = resolve_model_drop([first, first, second])
            self.assertFalse(resolution.is_importable)
            self.assertEqual(len(resolution.model_paths), 2)

    def test_mixed_supported_and_unsupported_drop_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            model = Path(temp_dir, "a.step")
            unsupported = Path(temp_dir, "mesh.stl")
            model.touch()
            unsupported.touch()
            resolution = resolve_model_drop([model, unsupported])
            self.assertFalse(resolution.is_importable)
            self.assertEqual(len(resolution.model_paths), 1)
            self.assertEqual(len(resolution.issues), 1)

    def test_unsupported_and_missing_files_have_actionable_issues(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            unsupported = Path(temp_dir, "mesh.stl")
            unsupported.touch()
            missing = Path(temp_dir, "missing.step")
            resolution = resolve_model_drop([unsupported, missing])
            self.assertFalse(resolution.is_importable)
            self.assertEqual(len(resolution.issues), 2)
            self.assertIn("Unsupported format", resolution.issues[0].reason)
            self.assertIn("does not exist", resolution.issues[1].reason)

    def test_mime_urls_take_priority_over_duplicate_text_uri(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            model = Path(temp_dir, "part.step")
            model.touch()
            url = FakeUrl(str(model), model.as_uri())
            mime = FakeMimeData([url], model.as_uri())
            self.assertEqual(sources_from_mime_data(mime), (url,))
            resolution = resolve_mime_model_drop(mime)
            self.assertTrue(resolution.is_importable)

    def test_plain_text_supports_newline_delimited_paths_not_space_split(self):
        mime = FakeMimeData(text='"C:\\Models With Spaces\\part.step"\nD:\\b.iges')
        self.assertEqual(
            sources_from_mime_data(mime),
            ('"C:\\Models With Spaces\\part.step"', 'D:\\b.iges'))

    def test_same_installed_filter_uses_language_active_at_each_event(self):
        manager = TranslationManager()
        manager.set_language("en")
        statuses = []
        imported = []
        viewer = FakeViewer()

        with tempfile.TemporaryDirectory() as temp_dir:
            model = Path(temp_dir, "part.step")
            model.touch()
            mime = FakeMimeData(urls=[FakeUrl(str(model))])
            event_filter = install_model_drop_support(
                FakeQtCore,
                viewer,
                on_model_path=imported.append,
                set_status=statuses.append,
                translate=manager.translate,
            )

            event_filter.eventFilter(
                viewer, FakeDropEvent(FakeQEvent.DragEnter, mime))
            self.assertEqual(statuses[-1], "Release to import model: part.step")

            manager.set_language("zh")
            # The filter and cached resolution are reused; only the active
            # translation changes at event-render time.
            event_filter.eventFilter(
                viewer, FakeDropEvent(FakeQEvent.DragMove, mime))
            self.assertEqual(statuses[-1], "释放鼠标以导入模型：part.step")

            event_filter.eventFilter(
                viewer, FakeDropEvent(FakeQEvent.Drop, mime))
            self.assertEqual(statuses[-1], "正在加载拖入的模型：part.step")
            self.assertEqual(imported, [str(model)])

        self.assertTrue(viewer.accept_drops)
        self.assertIs(viewer.event_filter, event_filter)

    def test_cached_rejection_status_and_error_follow_runtime_language(self):
        manager = TranslationManager()
        statuses = []
        errors = []
        viewer = FakeViewer()

        with tempfile.TemporaryDirectory() as temp_dir:
            unsupported = Path(temp_dir, "mesh.stl")
            unsupported.touch()
            mime = FakeMimeData(urls=[FakeUrl(str(unsupported))])
            event_filter = install_model_drop_support(
                FakeQtCore,
                viewer,
                on_model_path=lambda _path: None,
                show_error=errors.append,
                set_status=statuses.append,
                translate=manager.translate,
            )
            event_filter.eventFilter(
                viewer, FakeDropEvent(FakeQEvent.DragEnter, mime))
            self.assertIn("Unsupported format .stl", statuses[-1])

            manager.set_language("zh")
            event_filter.eventFilter(
                viewer, FakeDropEvent(FakeQEvent.Drop, mime))

        self.assertIn("格式 .stl 不受支持", statuses[-1])
        self.assertIn("无法导入拖入项", errors[-1])
        self.assertIn("mesh.stl", errors[-1])

    def test_chinese_rejection_keeps_dynamic_filename_and_extension(self):
        manager = TranslationManager(default_locale="zh")
        with tempfile.TemporaryDirectory() as temp_dir:
            unsupported = Path(temp_dir, "mesh.stl")
            unsupported.touch()
            resolution = resolve_model_drop([unsupported])
            feedback = format_drop_feedback(
                resolution, translate=manager.translate)
            detail = format_drop_error(
                resolution, translate=manager.translate)
        self.assertIn("mesh.stl", feedback)
        self.assertIn(".stl", feedback)
        self.assertIn("无法导入拖入项", detail)
        self.assertIn("mesh.stl", detail)
        self.assertIn(".stl", detail)

    def test_lightweight_translator_and_missing_key_use_safe_fallback(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            model = Path(temp_dir, "part.step")
            model.touch()
            resolution = resolve_model_drop([model])

        def lightweight(_key, filename):
            return "自定义：{}".format(filename)

        self.assertEqual(
            format_drop_feedback(resolution, translate=lightweight),
            "自定义：part.step")
        self.assertEqual(
            format_drop_feedback(
                resolution, translate=lambda _key, **_values: _key),
            "Release to import model: part.step")


if __name__ == "__main__":
    unittest.main()
