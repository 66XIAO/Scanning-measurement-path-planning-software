"""Colors shared by immediate OCC drawing and full scene redraws."""
from OCC.Display.OCCViewer import rgb_color

MODEL = rgb_color(0.78, 0.81, 0.83)
PATCHES = (
    rgb_color(0.76, 0.82, 0.84), rgb_color(0.81, 0.83, 0.79),
    rgb_color(0.79, 0.81, 0.87), rgb_color(0.84, 0.82, 0.78),
)
SELECTED = rgb_color(0.35, 0.70, 0.78)
CENTER = rgb_color(0.92, 0.63, 0.23)
CANDIDATE = rgb_color(0.21, 0.48, 0.75)
OPTIMAL = rgb_color(0.20, 0.64, 0.42)
PATH = rgb_color(0.56, 0.32, 0.67)
SENSOR = rgb_color(0.13, 0.62, 0.65)
ERROR = rgb_color(0.79, 0.28, 0.26)


def configure_viewer(display):
    """Use a neutral viewport when supported by the installed pythonOCC."""
    try:
        from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
        top = Quantity_Color(0.91, 0.93, 0.94, Quantity_TOC_RGB)
        bottom = Quantity_Color(0.76, 0.80, 0.83, Quantity_TOC_RGB)
        display.View.SetBgGradientColors(top, bottom, 2, True)
    except (AttributeError, TypeError):
        try:
            display.View.SetBackgroundColor(top)
        except (AttributeError, TypeError, UnboundLocalError):
            pass
    try:
        from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
        hover = display.Context.HighlightStyle()
        hover.SetColor(Quantity_Color(0.48, 0.75, 0.81, Quantity_TOC_RGB))
        selected = display.Context.SelectionStyle()
        selected.SetColor(Quantity_Color(0.35, 0.70, 0.78, Quantity_TOC_RGB))
    except (AttributeError, TypeError):
        pass
