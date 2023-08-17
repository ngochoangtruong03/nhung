#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_read_clicked();
    void portRead();
    void on_write_clicked();
    void on_clear_clicked();

    QString str2hexstr(QString, qint8);

    qint8 str2int(QString);

    QString decToBinary(qint8, qint8);

    QString convert_addr(qint8);

private:
    Ui::MainWindow *ui;

    QSerialPort *port;
};

#endif // MAINWINDOW_H
