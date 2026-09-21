"""Exercise the real Qt speed dialog offscreen without opening the OCC main viewer."""
import os
os.environ['QT_QPA_PLATFORM']='offscreen'
import faulthandler
faulthandler.dump_traceback_later(20)
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from OCC.Display import backend
print('Loading Qt backend',flush=True)
backend.load_backend('pyside6')
from PySide6 import QtWidgets, QtGui
import speed_planning_ui as ui


def main():
    print('Constructing QApplication',flush=True)
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    font_path=Path('C:/Windows/Fonts/msyh.ttc')
    if font_path.exists():
        font_id=QtGui.QFontDatabase.addApplicationFont(str(font_path))
        families=QtGui.QFontDatabase.applicationFontFamilies(font_id)
        if families:
            app.setFont(QtGui.QFont(families[0],10))
    out=Path('artifacts/speed_v1/dialog_check'); out.mkdir(exist_ok=True)
    def accept(dialog):
        combo=dialog.findChildren(QtWidgets.QComboBox)[0]
        assert combo.currentData()=='continuous_yang'
        dialog.show(); app.processEvents()
        dialog.grab().save(str(out/'continuous_dialog.png'))
        return QtWidgets.QDialog.Accepted
    ui._exec=accept
    result=ui.get_speed_planning_settings()
    assert result['start_speed']==result['end_speed']==0
    assert result['min_linear_speed']==.1
    assert result['position_tolerance_mm']==1.5
    assert result['orientation_tolerance_deg']==2.
    assert result['rounding_mm']==.3
    (out/'settings.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('Qt dialog settings and rendering verified')
    app.closeAllWindows()
    faulthandler.cancel_dump_traceback_later()


if __name__=='__main__': main()
