import json
from pathlib import Path
import tempfile
import unittest
import warnings

from i18n import (
    LocaleValidationError,
    TranslationManager,
    load_catalog_file,
)


def _catalog(locale, strings, aliases=None, name=None, native_name=None):
    return {
        "meta": {
            "locale": locale,
            "name": name or locale,
            "native_name": native_name or locale,
            "aliases": aliases or [],
        },
        "strings": strings,
    }


def _write_catalog(directory, filename, data):
    path = Path(directory) / filename
    with path.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
    return path


class BundledCatalogTests(unittest.TestCase):
    def test_bundled_english_and_chinese_catalogs_are_valid_utf8(self):
        manager = TranslationManager()
        languages = {info.code: info for info in manager.available_languages()}
        self.assertIn("en", languages)
        self.assertIn("zh_CN", languages)
        self.assertEqual(manager.invalid_catalogs(), {})

        manager.set_language("zh-CN")
        self.assertEqual(manager.language, "zh_CN")
        self.assertEqual(manager.translate("menu.file"), "文件")
        self.assertEqual(
            manager.translate("workflow.path_points", count=47),
            "路径点数：47")

    def test_bundled_chinese_catalog_covers_every_english_key(self):
        locales_dir = Path(__file__).resolve().parents[1] / "locales"
        english = load_catalog_file(locales_dir / "en.json")
        chinese = load_catalog_file(
            locales_dir / "zh_CN.json", fallback_strings=english.strings)
        self.assertEqual(set(chinese.strings), set(english.strings))

    def test_alias_and_safe_unknown_key_fallback(self):
        manager = TranslationManager()
        manager.set_language("zh")
        self.assertEqual(manager.translate("does.not.exist"), "does.not.exist")
        self.assertEqual(
            manager.translate("does.not.exist", default="Safe default"),
            "Safe default")


class TranslationManagerTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        _write_catalog(
            self.temp_dir.name,
            "en.json",
            _catalog("en", {
                "only.fallback": "English fallback",
                "welcome": "Hello {name}",
            }),
        )

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_missing_active_key_falls_back_to_english(self):
        _write_catalog(
            self.temp_dir.name,
            "zh_CN.json",
            _catalog("zh_CN", {"welcome": "你好，{name}"}, aliases=["zh"]),
        )
        manager = TranslationManager(self.temp_dir.name)
        manager.set_language("zh")
        self.assertEqual(manager.translate("only.fallback"), "English fallback")
        self.assertEqual(manager.translate("welcome", name="小王"), "你好，小王")

    def test_placeholder_mismatch_is_rejected_without_language_change(self):
        _write_catalog(
            self.temp_dir.name,
            "zh.json",
            _catalog("zh", {"welcome": "你好，{person}"}),
        )
        manager = TranslationManager(self.temp_dir.name)
        with self.assertRaises(LocaleValidationError):
            manager.set_language("zh")
        self.assertEqual(manager.language, "en")
        self.assertEqual(manager.translate("welcome", name="Ada"), "Hello Ada")

    def test_bad_runtime_format_values_do_not_interrupt_caller(self):
        manager = TranslationManager(self.temp_dir.name)
        self.assertEqual(manager.translate("welcome", unexpected="Ada"),
                         "Hello {name}")

    def test_listener_failure_is_isolated_and_unsubscribe_works(self):
        _write_catalog(
            self.temp_dir.name,
            "zh.json",
            _catalog("zh", {"welcome": "你好，{name}"}),
        )
        manager = TranslationManager(self.temp_dir.name)
        calls = []
        unsubscribe = manager.subscribe(calls.append)
        unsubscribe_failing = manager.subscribe(lambda _locale: 1 / 0)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            manager.set_language("zh")
        self.assertEqual(calls, ["zh"])
        self.assertTrue(any("listener failed" in str(item.message)
                            for item in caught))
        unsubscribe()
        unsubscribe_failing()
        manager.set_language("en")
        self.assertEqual(calls, ["zh"])

    def test_explicit_external_json_file_can_be_activated(self):
        manager = TranslationManager(self.temp_dir.name)
        _write_catalog(
            self.temp_dir.name,
            "zh.json",
            _catalog("zh_CN", {"welcome": "你好，{name}"}, aliases=["zh"]),
        )
        # Discovery registers the built-in zh catalog first.  A user-selected
        # file with the same canonical locale should still be able to override
        # it after full validation.
        manager.available_languages()
        with tempfile.TemporaryDirectory() as external_dir:
            path = _write_catalog(
                external_dir,
                "custom.json",
                _catalog("zh_CN", {"welcome": "您好，{name}"}),
            )
            manager.set_language_from_file(path)
        self.assertEqual(manager.language, "zh_CN")
        self.assertEqual(manager.translate("welcome", name="李工"), "您好，李工")


class CatalogValidationTests(unittest.TestCase):
    def test_duplicate_json_key_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.json"
            path.write_text(
                '{"meta":{"locale":"en","name":"English",'
                '"native_name":"English"},"strings":{"key":"one",'
                '"key":"two"}}',
                encoding="utf-8")
            with self.assertRaises(LocaleValidationError):
                load_catalog_file(path)

    def test_non_string_translation_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = _write_catalog(
                directory, "bad.json", _catalog("en", {"bad": 123}))
            with self.assertRaises(LocaleValidationError):
                load_catalog_file(path)

    def test_invalid_catalog_is_reported_but_not_listed(self):
        with tempfile.TemporaryDirectory() as directory:
            _write_catalog(directory, "en.json", _catalog("en", {"ok": "OK"}))
            (Path(directory) / "broken.json").write_text("{broken", encoding="utf-8")
            manager = TranslationManager(directory)
            self.assertEqual([info.code for info in manager.available_languages()], ["en"])
            self.assertEqual(len(manager.invalid_catalogs()), 1)


if __name__ == "__main__":
    unittest.main()
