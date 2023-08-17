#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <QDebug>
#include <QMessageBox>
#include <bits/stdc++.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    port = new QSerialPort(this);
    port->setPortName("COM6");
    port->setBaudRate(QSerialPort::Baud115200);
    port->setDataBits(QSerialPort::Data8);
    port->setParity(QSerialPort::NoParity);
    port->setStopBits(QSerialPort::OneStop);

    port->open(QIODevice::ReadWrite);
    connect(port, SIGNAL(readyRead()), this, SLOT(portRead()));
}

MainWindow::~MainWindow()
{
    delete ui;
    port->close();
    delete port;
}

void MainWindow::on_read_clicked()
{
    QByteArray cmd;
    cmd.append('r');
    cmd.append(str2hexstr(ui->lineEdit->text().toStdString().data(), 3));   // page
    cmd.append(str2hexstr(ui->lineEdit_2->text().toStdString().data(), 2)); // offset
    cmd.append(str2hexstr(ui->lineEdit_4->text().toStdString().data(), 2)); // size
    // data: don't care

    //qDebug()<<cmd;
    port->write(cmd);
    port->waitForBytesWritten(200);
}

void MainWindow::on_write_clicked()
{
    QByteArray cmd;
    cmd.append('w');
    cmd.append(str2hexstr(ui->lineEdit->text().toStdString().data(), 3));   // page
    cmd.append(str2hexstr(ui->lineEdit_2->text().toStdString().data(), 2)); // offset
    cmd.append(str2hexstr(ui->lineEdit_4->text().toStdString().data(), 2)); // size
    cmd.append(ui->lineEdit_3->text().toStdString().data());                // data

    port->write(cmd);
    port->waitForBytesWritten(200);

    QMessageBox::about(this, "Write data", "Write data to eeprom is successfully!!!");
}

void MainWindow::on_clear_clicked()
{
    QByteArray cmd;
    cmd.append('c');

    port->write(cmd);
    port->waitForBytesWritten(200);

    QMessageBox::about(this, "Clear data", "Clear data successfully!!!");
}


void MainWindow::portRead()
{
    QByteArray data = port->readAll();
    QByteArray show;
    qint8 size = str2int(ui->lineEdit_4->text().toStdString().data());

    show.append("\naddress                    value                     No\n");
    for (qint8 i = 0; i < size; i++) {
        show.append(convert_addr(i));
        show.append("                           ");
        show.append(data[i]);
        show.append("                        ");
        QString s = QString::number(i);
        show.append(s);
        show.append("\n");
    }

    ui->showData->moveCursor(QTextCursor::End);
    ui->showData->insertPlainText(show);
}

QString MainWindow::str2hexstr(QString str, qint8 bits)
{
    qint8 iValue = str.toInt();
    QString hexstr = QString::number(iValue, 16).toUpper();
    while(hexstr.size() < bits) hexstr = '0' + hexstr;

    return hexstr;
}

qint8 MainWindow::str2int(QString s)
{
    qint8 i = s.toInt();
    return i;
}

QString MainWindow::decToBinary(qint8 n, qint8 num)
{
    QString binary;

    for (int i = 0; i < num; i++) {
        int bi = n % 2;
        QString biStr = QString::number(bi);
        binary += biStr;
        n = n / 2;
    }

    std::reverse(binary.begin(), binary.end());

    return binary;
}

QString MainWindow::convert_addr(qint8 i)
{
    QString addr_bi = "0";
    qint8 page = str2int(ui->lineEdit->text().toStdString().data());
    qint8 off = str2int(ui->lineEdit_2->text().toStdString().data());

    while (off + i >= 64){
        page += 1;
        off -= 64;
    }
    off += i;

    addr_bi.append(decToBinary(page, 9));
    addr_bi.append(decToBinary(off, 6));

    QString addr_hex = "0x";

    bool fOK;
    int iValue = addr_bi.toInt(&fOK, 2);  //2 is the base

    addr_hex = QString::number(iValue, 16).toUpper();
    while(addr_hex.size() < 4) addr_hex = '0' + addr_hex;
    addr_hex = 'x' + addr_hex;
    //qDebug() << addr_hex;

    return addr_hex;
}
