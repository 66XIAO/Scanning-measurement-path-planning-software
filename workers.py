def create_background_worker_class(QtCore):
    Signal = getattr(QtCore, "pyqtSignal", getattr(QtCore, "Signal", None))
    if Signal is None:
        raise RuntimeError("The selected Qt binding does not expose Signal/pyqtSignal")

    class BackgroundWorker(QtCore.QThread):
        finished_with_result = Signal(object)
        failed_with_error = Signal(str)

        def __init__(self, fn, parent=None):
            super().__init__(parent)
            self.fn = fn

        def run(self):
            try:
                self.finished_with_result.emit(self.fn())
            except Exception as e:
                import traceback
                self.failed_with_error.emit("{}\n{}".format(str(e), traceback.format_exc()))

    return BackgroundWorker


class TaskRunner:
    def __init__(self, QtCore, QtWidgets, get_parent):
        self.QtCore = QtCore
        self.QtWidgets = QtWidgets
        self.get_parent = get_parent
        self.BackgroundWorker = create_background_worker_class(QtCore)
        self.active_workers = []
        self.active_progress_dialogs = []

    def run(self, title, message, work_fn, success_fn, error_fn=None):
        parent = self.get_parent()
        progress = self.QtWidgets.QProgressDialog(message, None, 0, 0, parent)
        progress.setWindowTitle(title)
        progress.setWindowModality(self.QtCore.Qt.WindowModal)
        progress.setMinimumDuration(0)
        progress.setAutoClose(False)
        progress.setAutoReset(False)
        progress.show()

        worker = self.BackgroundWorker(work_fn, parent)
        self.active_workers.append(worker)
        self.active_progress_dialogs.append(progress)

        def cleanup():
            progress.close()
            if worker in self.active_workers:
                self.active_workers.remove(worker)
            if progress in self.active_progress_dialogs:
                self.active_progress_dialogs.remove(progress)
            worker.deleteLater()

        def success(result):
            cleanup()
            success_fn(result)

        def failure(error_text):
            cleanup()
            if error_fn:
                error_fn(error_text)

        worker.finished_with_result.connect(success)
        worker.failed_with_error.connect(failure)
        worker.start()
        return worker
