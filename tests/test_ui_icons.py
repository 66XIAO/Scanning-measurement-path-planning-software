import importlib
from pathlib import Path
import tempfile
import types
import unittest

from runtime_environment import select_qt_binding
import ui_theme


class UIIconRenderingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        binding = select_qt_binding()
        cls.gui = importlib.import_module(binding + '.QtGui')
        cls.widgets = importlib.import_module(binding + '.QtWidgets')
        cls.app = cls.widgets.QApplication.instance() or cls.widgets.QApplication([])

    def assert_visible(self, icon, size):
        pixmap = icon.pixmap(size, size)
        self.assertFalse(pixmap.isNull())
        image = pixmap.toImage()
        self.assertTrue(any(image.pixelColor(x, y).alpha() > 0
                            for y in range(image.height())
                            for x in range(image.width())))

    def broken_plugin_gui(self):
        gui = self.gui

        class BrokenSvgIcon:
            def pixmap(self, *args):
                return gui.QPixmap()

        facade = types.ModuleType(gui.__name__)
        facade.QIcon = lambda path=None: BrokenSvgIcon() if path else gui.QIcon()
        for name in ('QPixmap', 'QColor', 'QPainter'):
            setattr(facade, name, getattr(gui, name))
        return facade

    def test_every_mapped_asset_has_visible_pixels_at_ribbon_and_hidpi_sizes(self):
        for name in sorted(set(ui_theme.ACTION_ICONS.values()) | {'app'}):
            for size in (20, 28, 36, 56, 72):
                with self.subTest(icon=name, size=size):
                    self.assert_visible(ui_theme.load_icon(self.gui, name), size)

    def test_native_plugin_failure_uses_same_svg_with_transparency(self):
        icon = ui_theme.load_icon(self.broken_plugin_gui(), 'import-model')
        for size in (16, 32, 80, 256):
            self.assert_visible(icon, size)
        self.assertEqual(icon.pixmap(32, 32).toImage().pixelColor(0, 0).alpha(), 0)

    def test_missing_asset_reports_its_path(self):
        with self.assertRaisesRegex(RuntimeError, 'UI icon asset is missing'):
            ui_theme.load_icon(self.gui, 'missing-regression-test-icon')

    def test_invalid_svg_is_not_silently_accepted_by_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.svg'
            path.write_text('<svg>invalid', encoding='utf-8')
            with self.assertRaisesRegex(RuntimeError, 'Invalid SVG icon asset'):
                ui_theme._load_svg_icon(self.broken_plugin_gui(), str(path))


if __name__ == '__main__':
    unittest.main()
