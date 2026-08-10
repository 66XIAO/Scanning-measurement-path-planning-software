"""Small, Qt-independent JSON translation service for the desktop UI.

The service deliberately owns only translated text.  It never recreates
widgets and never mutates :class:`state.AppState`, so changing language can be
performed while a model, path, or speed plan is already loaded.

Catalog format::

    {
      "meta": {
        "locale": "en",
        "name": "English",
        "native_name": "English",
        "aliases": ["en-US"]
      },
      "strings": {
        "app.title": "Application title",
        "message.loaded": "Loaded {filename}"
      }
    }

Translations use named ``str.format`` fields.  Catalogs are validated before
activation and a failed switch leaves the current language unchanged.
"""

from dataclasses import dataclass
import json
from pathlib import Path
import re
from string import Formatter
import threading
import warnings


DEFAULT_LOCALES_DIR = Path(__file__).resolve().with_name("locales")
DEFAULT_LOCALE = "en"
_LOCALE_RE = re.compile(r"^[A-Za-z][A-Za-z0-9]*(?:[-_][A-Za-z0-9]+)*$")
_KEY_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


class LocaleValidationError(ValueError):
    """Raised when a locale file is missing, malformed, or inconsistent."""


@dataclass(frozen=True)
class LocaleInfo:
    """Metadata displayed by a language-selection menu."""

    code: str
    name: str
    native_name: str
    aliases: tuple = ()


@dataclass(frozen=True)
class _Catalog:
    info: LocaleInfo
    strings: dict
    source: Path


def _normalise_locale(code):
    if not isinstance(code, str) or not code.strip():
        raise LocaleValidationError("Locale code must be a non-empty string")
    code = code.strip()
    if not _LOCALE_RE.fullmatch(code):
        raise LocaleValidationError("Invalid locale code: {!r}".format(code))
    return code.replace("_", "-").lower()


def _reject_duplicate_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise LocaleValidationError(
                "Duplicate JSON key {!r} in locale catalog".format(key))
        result[key] = value
    return result


def _format_fields(template, key):
    try:
        parsed = Formatter().parse(template)
        fields = []
        for _literal, field_name, _format_spec, _conversion in parsed:
            if field_name is None:
                continue
            # Positional, attribute, and index lookups make translation files
            # fragile.  Named fields provide a stable contract across locales.
            if not field_name.isidentifier():
                raise LocaleValidationError(
                    "Translation {!r} has invalid format field {!r}".format(
                        key, field_name))
            fields.append(field_name)
        return tuple(sorted(fields))
    except ValueError as exc:
        raise LocaleValidationError(
            "Translation {!r} has invalid format syntax: {}".format(key, exc))


def _validate_catalog_data(data, source, fallback_strings=None):
    if not isinstance(data, dict):
        raise LocaleValidationError(
            "Locale catalog root must be a JSON object: {}".format(source))

    meta = data.get("meta")
    strings = data.get("strings")
    if not isinstance(meta, dict):
        raise LocaleValidationError(
            "Locale catalog requires a 'meta' object: {}".format(source))
    if not isinstance(strings, dict):
        raise LocaleValidationError(
            "Locale catalog requires a 'strings' object: {}".format(source))

    code = meta.get("locale")
    _normalise_locale(code)
    name = meta.get("name")
    native_name = meta.get("native_name")
    if not isinstance(name, str) or not name.strip():
        raise LocaleValidationError("meta.name must be a non-empty string")
    if not isinstance(native_name, str) or not native_name.strip():
        raise LocaleValidationError(
            "meta.native_name must be a non-empty string")

    aliases = meta.get("aliases", [])
    if not isinstance(aliases, list) or not all(
            isinstance(alias, str) for alias in aliases):
        raise LocaleValidationError("meta.aliases must be an array of strings")
    normalised_codes = {_normalise_locale(code)}
    for alias in aliases:
        normalised = _normalise_locale(alias)
        if normalised in normalised_codes:
            raise LocaleValidationError(
                "Duplicate locale code/alias: {!r}".format(alias))
        normalised_codes.add(normalised)

    checked_strings = {}
    for key, value in strings.items():
        if not isinstance(key, str) or not _KEY_RE.fullmatch(key):
            raise LocaleValidationError(
                "Invalid translation key {!r} in {}".format(key, source))
        if not isinstance(value, str):
            raise LocaleValidationError(
                "Translation {!r} must be a string".format(key))
        fields = _format_fields(value, key)
        if fallback_strings is not None and key in fallback_strings:
            fallback_fields = _format_fields(fallback_strings[key], key)
            if fields != fallback_fields:
                raise LocaleValidationError(
                    "Translation {!r} fields {} do not match fallback fields {}"
                    .format(key, fields, fallback_fields))
        checked_strings[key] = value

    return _Catalog(
        info=LocaleInfo(code.strip(), name.strip(), native_name.strip(),
                        tuple(alias.strip() for alias in aliases)),
        strings=checked_strings,
        source=Path(source),
    )


def load_catalog_file(path, fallback_strings=None):
    """Load and validate one UTF-8 JSON locale file.

    ``fallback_strings`` is optional.  When provided, placeholders in every
    translated value must match the corresponding fallback value.
    """

    path = Path(path).resolve()
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = json.load(stream, object_pairs_hook=_reject_duplicate_pairs)
    except LocaleValidationError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise LocaleValidationError(
            "Cannot load locale catalog {}: {}".format(path, exc))
    return _validate_catalog_data(data, path, fallback_strings)


class TranslationManager:
    """Manage validated catalogs and notify the UI after atomic switches."""

    def __init__(self, locales_dir=None, default_locale=DEFAULT_LOCALE,
                 fallback_locale=DEFAULT_LOCALE):
        self.locales_dir = Path(locales_dir or DEFAULT_LOCALES_DIR).resolve()
        if not self.locales_dir.is_dir():
            raise LocaleValidationError(
                "Locales directory does not exist: {}".format(self.locales_dir))

        self._lock = threading.RLock()
        self._listeners = []
        self._catalogs = {}
        self._invalid_catalogs = {}

        fallback = self._find_and_load(fallback_locale, fallback_strings=None)
        self._fallback = fallback
        self._register_catalog(fallback)
        if (_normalise_locale(default_locale)
                == _normalise_locale(fallback.info.code)):
            self._active = fallback
        else:
            self._active = self._find_and_load(
                default_locale, fallback_strings=fallback.strings)
            self._register_catalog(self._active)

    @property
    def language(self):
        with self._lock:
            return self._active.info.code

    @property
    def language_info(self):
        with self._lock:
            return self._active.info

    @property
    def fallback_language(self):
        return self._fallback.info.code

    def _register_catalog(self, catalog):
        codes = (catalog.info.code,) + catalog.info.aliases
        for code in codes:
            normalised = _normalise_locale(code)
            existing = self._catalogs.get(normalised)
            if existing is not None and existing.source != catalog.source:
                raise LocaleValidationError(
                    "Locale alias {!r} is declared by both {} and {}".format(
                        code, existing.source, catalog.source))
            self._catalogs[normalised] = catalog

    def _candidate_paths(self, locale):
        normalised = _normalise_locale(locale)
        raw = locale.strip()
        names = [raw, raw.replace("-", "_"), normalised,
                 normalised.replace("-", "_")]
        seen = set()
        for name in names:
            folded = name.lower()
            if folded in seen:
                continue
            seen.add(folded)
            yield self.locales_dir / (name + ".json")

    def _load_path(self, path, fallback_strings):
        catalog = load_catalog_file(path, fallback_strings=fallback_strings)
        return catalog

    def _find_and_load(self, locale, fallback_strings):
        normalised = _normalise_locale(locale)
        cached = self._catalogs.get(normalised)
        if cached is not None:
            return cached

        # Prefer a direct filename.  This makes a malformed requested catalog
        # fail explicitly instead of looking like an unknown language.
        for candidate in self._candidate_paths(locale):
            if candidate.is_file():
                catalog = self._load_path(candidate, fallback_strings)
                codes = (catalog.info.code,) + catalog.info.aliases
                if normalised not in {_normalise_locale(code) for code in codes}:
                    raise LocaleValidationError(
                        "Requested locale {!r} does not match catalog metadata {}"
                        .format(locale, candidate))
                return catalog

        # Aliases such as ``zh`` may not match the filename ``zh_CN.json``.
        for path in sorted(self.locales_dir.glob("*.json")):
            try:
                catalog = self._load_path(path, fallback_strings)
                self._register_catalog(catalog)
            except LocaleValidationError as exc:
                self._invalid_catalogs[path] = str(exc)
                continue
            if normalised in {
                    _normalise_locale(catalog.info.code),
                    *(_normalise_locale(alias) for alias in catalog.info.aliases),
            }:
                return catalog

        raise LocaleValidationError(
            "Unknown locale {!r}; no matching JSON catalog in {}".format(
                locale, self.locales_dir))

    def available_languages(self):
        """Return metadata for all valid catalogs, sorted by locale code."""

        with self._lock:
            fallback_strings = self._fallback.strings
            for path in sorted(self.locales_dir.glob("*.json")):
                try:
                    catalog = self._load_path(path, fallback_strings)
                    self._register_catalog(catalog)
                    self._invalid_catalogs.pop(path, None)
                except LocaleValidationError as exc:
                    self._invalid_catalogs[path] = str(exc)
            unique = {catalog.source: catalog
                      for catalog in self._catalogs.values()}
            return tuple(sorted(
                (catalog.info for catalog in unique.values()),
                key=lambda info: _normalise_locale(info.code)))

    def invalid_catalogs(self):
        """Return ``{path: error}`` for invalid files found during discovery."""

        self.available_languages()
        with self._lock:
            return dict(self._invalid_catalogs)

    def set_language(self, locale):
        """Validate and atomically activate ``locale``.

        Listener failures are isolated so one UI component cannot prevent the
        remaining components from retranslating.
        """

        catalog = self._find_and_load(
            locale, fallback_strings=self._fallback.strings)
        self._register_catalog(catalog)
        return self._activate(catalog)

    def set_language_from_file(self, path):
        """Validate and activate an explicitly selected JSON catalog file."""

        catalog = self._load_path(path, self._fallback.strings)
        return self._activate(catalog)

    def _activate(self, catalog):
        with self._lock:
            if (self._active.source == catalog.source
                    and self._active.strings == catalog.strings):
                return catalog.info
            self._active = catalog
            listeners = tuple(self._listeners)

        for callback in listeners:
            try:
                callback(catalog.info.code)
            except Exception as exc:  # UI listeners must not break switching.
                warnings.warn(
                    "Language-change listener failed: {}".format(exc),
                    RuntimeWarning)
        return catalog.info

    def subscribe(self, callback):
        """Register ``callback(locale_code)`` and return an unsubscribe hook."""

        if not callable(callback):
            raise TypeError("Language-change listener must be callable")
        with self._lock:
            if callback not in self._listeners:
                self._listeners.append(callback)

        def unsubscribe():
            with self._lock:
                if callback in self._listeners:
                    self._listeners.remove(callback)

        return unsubscribe

    def translate(self, key, default=None, **values):
        """Translate ``key`` with active -> fallback -> key safe fallback."""

        if not isinstance(key, str) or not key:
            raise TypeError("Translation key must be a non-empty string")
        with self._lock:
            template = self._active.strings.get(key)
            if template is None:
                template = self._fallback.strings.get(key)
        if template is None:
            template = key if default is None else default
        if not isinstance(template, str):
            raise TypeError("Translation default must be a string")
        if not values:
            return template
        try:
            return template.format(**values)
        except (KeyError, IndexError, ValueError, AttributeError):
            # A missing runtime value should never terminate an in-progress UI
            # workflow.  Returning the validated template is the safest signal.
            return template

    __call__ = translate


_default_manager = None
_default_lock = threading.Lock()


def get_manager():
    """Return the process-wide translation manager (created lazily)."""

    global _default_manager
    if _default_manager is None:
        with _default_lock:
            if _default_manager is None:
                _default_manager = TranslationManager()
    return _default_manager


def tr(key, default=None, **values):
    """Translate using the process-wide manager."""

    return get_manager().translate(key, default=default, **values)


def set_language(locale):
    """Activate a built-in locale on the process-wide manager."""

    return get_manager().set_language(locale)


def set_language_from_file(path):
    """Activate a selected JSON locale on the process-wide manager."""

    return get_manager().set_language_from_file(path)


def available_languages():
    return get_manager().available_languages()


def get_language():
    return get_manager().language


def subscribe_language_changed(callback):
    return get_manager().subscribe(callback)
