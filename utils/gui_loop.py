"""
https://github.com/nppi3enz/dollyproject-pyqt/blob/master/guiLoop.py
"""
from PyQt5.QtCore import QTimer


# def _loop_in_the_gui(gui_element, generator, _start_in_gui):
#     try:
#         # generator yields the time to wait
#         wait_time = next(generator)
#     except StopIteration:
#         pass
#     else:
#         if wait_time is None:
#             # yield
#             wait_time = 0
#         else:
#             # yield seconds
#             wait_time = int(wait_time * 1000)  # Tkinter works with milli seconds
#         call_this_again = lambda: _loop_in_the_gui(gui_element, generator, _start_in_gui)
#         _start_in_gui(gui_element, wait_time, call_this_again)
#
#
# def use_py_qt5_q_timer(gui_element, wait_time, call_this):
#     QTimer.singleShot(wait_time, call_this)

def _loop_in_the_gui(gui_element, generator):
    try:
        # generator yields the time to wait
        wait_time = next(generator)
    except StopIteration:
        pass
    else:
        if wait_time is None:
            # yield
            wait_time = 0
        else:
            # yield seconds
            wait_time = int(wait_time * 1000)  # Tkinter works with milli seconds
        call_this_again = lambda: _loop_in_the_gui(gui_element, generator)
        use_py_qt5_q_timer(wait_time, call_this_again)


def use_py_qt5_q_timer(wait_time, call_this):
    QTimer.singleShot(wait_time, call_this)


# noinspection PyPep8Naming
class guiLoop(object):

    def __init__(self, function):
        """make a function to a guiLoop function
        The resulting function needs a gui element as first argument."""
        self.function = function
        self.__doc__ = function.__doc__
        self.__name__ = function.__name__
        # self.start_in_gui = start_in_gui

    def __call__(self, gui_element, *args, **kw):
        generator = self.function(*args, **kw)
        _loop_in_the_gui(gui_element, generator)
        return generator

    def __get__(self, gui_element, cls):
        if gui_element is None:
            return self
        return lambda *args, **kw: self(gui_element, gui_element, *args, **kw)
