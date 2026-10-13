"""Local desktop station for scp-copied record folders. No ROS required."""

import argparse
import os
from pathlib import Path
import sys
from datetime import datetime


def discover_sessions(directory):
    directory = Path(directory).expanduser().resolve()
    if (directory / 'manifest.json').is_file():
        return [directory]
    return sorted({path.parent for path in directory.rglob('manifest.json')
                   if any((path.parent / name).is_dir()
                          for name in ('metadata', 'video', 'bag', 'logs', 'camera'))}, reverse=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', nargs='?', type=Path)
    args = parser.parse_args(argv)
    from PySide6.QtCore import QProcess, QProcessEnvironment, QUrl
    from PySide6.QtGui import QDesktopServices
    from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFileDialog,
                                  QHBoxLayout, QLabel, QLineEdit, QMainWindow,
                                  QMessageBox, QPlainTextEdit, QProgressBar,
                                  QPushButton, QSpinBox, QVBoxLayout, QWidget)
    try:
        from PySide6.QtWebEngineWidgets import QWebEngineView
    except ImportError:
        QWebEngineView = None

    class ReviewWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle('AUV record 离线分析上位机')
            self.resize(1360, 940)
            self.report = None
            self.process = QProcess(self)
            self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
            self.process.readyReadStandardOutput.connect(self.read_output)
            self.process.finished.connect(self.finished)
            self.process.errorOccurred.connect(self.process_error)
            layout = QVBoxLayout()
            layout.addWidget(QLabel('选择 scp 到本机的 record 文件夹或单个 session；分析结果另存，原始录像保留。'))
            self.folder = QLineEdit()
            browse = QPushButton('选择 record 文件夹')
            browse.clicked.connect(self.choose_folder)
            row = QHBoxLayout()
            row.addWidget(self.folder, 1)
            row.addWidget(browse)
            scan = QPushButton('扫描会话')
            scan.clicked.connect(self.scan)
            row.addWidget(scan)
            layout.addLayout(row)
            self.sessions = QComboBox()
            layout.addWidget(self.sessions)
            self.output = QLineEdit(str(Path.home() / 'AUV_reports'))
            choose_output = QPushButton('选择报告保存目录')
            choose_output.clicked.connect(self.choose_output)
            row = QHBoxLayout()
            row.addWidget(QLabel('报告根目录'))
            row.addWidget(self.output, 1)
            row.addWidget(choose_output)
            layout.addLayout(row)
            self.tags = QCheckBox('录像离线 AprilTag 重识别')
            self.tags.setChecked(True)
            self.bag = QCheckBox('缺失 JSONL 时尝试 bag 补读（需本机 ROS 消息包）')
            self.limit = QSpinBox()
            self.limit.setRange(0, 100000)
            self.limit.setValue(200)
            row = QHBoxLayout()
            row.addWidget(self.tags)
            row.addWidget(QLabel('最多标注图片'))
            row.addWidget(self.limit)
            row.addWidget(self.bag)
            layout.addLayout(row)
            self.run = QPushButton('生成完整报告')
            self.run.clicked.connect(self.start_analysis)
            self.open_report = QPushButton('浏览器打开报告')
            self.open_report.setEnabled(False)
            self.open_report.clicked.connect(lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.report))))
            self.open_folder = QPushButton('打开导出文件夹')
            self.open_folder.setEnabled(False)
            self.open_folder.clicked.connect(lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.report.parent))))
            self.scene_button = QPushButton('三维九宫格')
            self.scene_button.setEnabled(False)
            self.scene_button.clicked.connect(self.show_scene)
            self.report_button = QPushButton('完整报告')
            self.report_button.setEnabled(False)
            self.report_button.clicked.connect(self.show_report)
            row = QHBoxLayout()
            for button in (self.run, self.scene_button, self.report_button, self.open_report, self.open_folder):
                row.addWidget(button)
            layout.addLayout(row)
            self.status = QLabel('等待选择 record 文件夹')
            layout.addWidget(self.status)
            self.progress = QProgressBar()
            self.progress.setRange(0, 1)
            layout.addWidget(self.progress)
            self.logs = QPlainTextEdit()
            self.logs.setReadOnly(True)
            self.logs.setMaximumBlockCount(3000)
            self.logs.setMaximumHeight(140)
            layout.addWidget(self.logs)
            self.viewer = QWebEngineView() if QWebEngineView else QLabel('报告生成后可点击“浏览器打开报告”查看交互图表和视频。')
            if QWebEngineView:
                self.viewer.page().profile().downloadRequested.connect(self.save_download)
            layout.addWidget(self.viewer, 1)
            container = QWidget()
            container.setLayout(layout)
            self.setCentralWidget(container)
            if args.directory:
                self.folder.setText(str(args.directory))
                self.scan()

        def choose_folder(self):
            path = QFileDialog.getExistingDirectory(self, '选择本地 record / session 文件夹', self.folder.text())
            if path:
                self.folder.setText(path)
                self.scan()

        def choose_output(self):
            path = QFileDialog.getExistingDirectory(self, '选择报告根目录', self.output.text())
            if path:
                self.output.setText(path)

        def scan(self):
            self.sessions.clear()
            path = Path(self.folder.text()).expanduser()
            if not self.folder.text().strip() or not path.is_dir():
                self.status.setText('请选择存在的本地文件夹')
                return
            try:
                for session in discover_sessions(path):
                    self.sessions.addItem(str(session), str(session))
                self.status.setText(f'找到 {self.sessions.count()} 个会话；选择一个后生成报告')
            except OSError as error:
                self.status.setText(str(error))

        def start_analysis(self):
            session = self.sessions.currentData()
            if not session:
                self.status.setText('未找到 session：需要 manifest.json 及 metadata/video 等录制目录')
                return
            if not self.output.text().strip():
                self.status.setText('请选择报告根目录')
                return
            output = Path(self.output.text()).expanduser().resolve() / (
                Path(session).name + '_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            if Path(session) == output or Path(session) in output.parents:
                self.status.setText('报告目录必须放在所选 session 外')
                return
            self.report = output / 'report.html'
            command = ['-u', '-m', 'uv_record.analyze', session, '--output', str(output),
                       '--max-tag-images', str(self.limit.value())]
            if not self.tags.isChecked():
                command.append('--no-detect-tags')
            if not self.bag.isChecked():
                command.append('--no-bag')
            environment = QProcessEnvironment.systemEnvironment()
            package_root = str(Path(__file__).resolve().parents[1])
            environment.insert('PYTHONPATH', package_root + os.pathsep + environment.value('PYTHONPATH'))
            self.process.setProcessEnvironment(environment)
            self.logs.clear()
            self.logs.appendPlainText(f'会话：{session}\n报告：{output}')
            self.run.setEnabled(False)
            self.open_report.setEnabled(False)
            self.open_folder.setEnabled(False)
            self.scene_button.setEnabled(False)
            self.report_button.setEnabled(False)
            self.progress.setRange(0, 0)
            self.status.setText('正在解析、导出视频和识别图像，大录像可能需要数分钟…')
            self.process.start(sys.executable, command)

        def read_output(self):
            self.logs.appendPlainText(bytes(self.process.readAllStandardOutput()).decode('utf-8', errors='replace').rstrip())

        def process_error(self, error):
            if error == QProcess.ProcessError.FailedToStart:
                self.finished(-1, QProcess.ExitStatus.CrashExit)

        def finished(self, code, status):
            self.read_output()
            self.run.setEnabled(True)
            self.progress.setRange(0, 1)
            success = code == 0 and self.report and self.report.is_file()
            self.progress.setValue(int(bool(success)))
            self.open_report.setEnabled(bool(success))
            self.scene_button.setEnabled(bool(success))
            self.report_button.setEnabled(bool(success))
            self.open_folder.setEnabled(bool(self.report and self.report.parent.is_dir()))
            self.status.setText('报告已生成：视觉、提交对照及完整文档见下方' if success else '分析失败；原因见日志，部分导出保留在报告目录')
            if success and QWebEngineView:
                self.show_scene()

        def show_scene(self):
            url = QUrl.fromLocalFile(str(self.report.parent / 'map3d.html'))
            if QWebEngineView:
                self.viewer.load(url)
            else:
                QDesktopServices.openUrl(url)

        def show_report(self):
            url = QUrl.fromLocalFile(str(self.report))
            if QWebEngineView:
                self.viewer.load(url)
            else:
                QDesktopServices.openUrl(url)

        def save_download(self, download):
            suggested = (self.report.parent if self.report else Path.home()) / download.suggestedFileName()
            path, _ = QFileDialog.getSaveFileName(self, '保存三维场景图片 / 导出文件', str(suggested))
            if path:
                download.setDownloadDirectory(str(Path(path).parent))
                download.setDownloadFileName(Path(path).name)
                download.accept()

        def closeEvent(self, event):
            if self.process.state() != QProcess.ProcessState.NotRunning:
                QMessageBox.information(self, '正在分析', '请等待当前分析完成后关闭窗口。')
                event.ignore()
            else:
                event.accept()

    app = QApplication(sys.argv[:1])
    window = ReviewWindow()
    window.show()
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
