/********************************************************************************
** Form generated from reading UI file 'siform.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SIFORM_H
#define UI_SIFORM_H

#include <Qt3Support/Q3ComboBox>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDial>
#include <QtGui/QDialog>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QTextBrowser>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SIForm
{
public:
    QTabWidget *tabWidget;
    QWidget *tab;
    QGroupBox *groupBox_DataCollection;
    QPushButton *DC_saveButton;
    QPushButton *DC_dictateButton;
    QPushButton *DC_newButton;
    QGroupBox *groupBox_Scale_P;
    QPushButton *scalePDecreaseButton;
    QPushButton *scalePIncreaseButton;
    QLineEdit *scalePLineEdit;
    QPushButton *okButton;
    QPushButton *connectionButton;
    QLabel *label_3;
    QPushButton *startButton;
    QLineEdit *lineEdit_Time;
    QPushButton *loginButton;
    QPushButton *logoButton;
    QLCDNumber *lcdcurrent;
    QLCDNumber *lcdnext;
    QLCDNumber *lcdtrial;
    QPushButton *connectionButton_2;
    QPushButton *connectionButton_3;
    QPushButton *connectionButton_4;
    QTextBrowser *textexpstatus;
    QTextBrowser *textgraspstatus;
    QGroupBox *groupBox_Scale_G;
    QPushButton *scaleGDecreaseButton;
    QPushButton *scaleGIncreaseButton;
    QLineEdit *scaleGLineEdit;
    QGroupBox *groupBox_master;
    QCheckBox *checkBox_GRIPPER;
    QCheckBox *checkBox_ORI;
    QCheckBox *checkBox_POS;
    QCheckBox *checkBox_fullscreen;
    QLCDNumber *lcdFPS;
    QGroupBox *groupBox_5;
    QDoubleSpinBox *zeroX;
    QDoubleSpinBox *zeroY;
    QDoubleSpinBox *zeroZ;
    QLabel *label_21;
    QLabel *label_22;
    QLabel *label_23;
    QPushButton *zerobutton;
    QWidget *tab2;
    QLabel *label_4;
    QLCDNumber *lCDNumber2;
    QPushButton *A_okButton;
    QPushButton *A_startButton;
    QComboBox *cB_IP;
    QTextEdit *infoText;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QDoubleSpinBox *camAngle1;
    QDoubleSpinBox *camAngle2;
    QDoubleSpinBox *camAngle3;
    QGroupBox *groupBox;
    QDoubleSpinBox *camLeftOffY;
    QDoubleSpinBox *camLeftOffX;
    QLabel *label;
    QLabel *label_12;
    QGroupBox *groupBox_2;
    QDoubleSpinBox *camRightOffX;
    QDoubleSpinBox *camRightOffY;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QGroupBox *groupBox_3;
    QDoubleSpinBox *camZoom;
    QLabel *label_2;
    QWidget *tab_2;
    QDial *dialDirection;
    Q3ComboBox *comboControlMode;
    QLabel *label_11;
    QSpinBox *spinDirection;
    QPushButton *autoMoveButtonGo;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QSpinBox *spinAmplitude;
    QSpinBox *spinDuration;
    QSpinBox *spinHold;
    QSpinBox *spinRepetition;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_16;
    QPushButton *autoMoveButtonStop;
    QSpinBox *spinZOffset;
    QLabel *label_39;

    void setupUi(QDialog *SIForm)
    {
        if (SIForm->objectName().isEmpty())
            SIForm->setObjectName(QString::fromUtf8("SIForm"));
        SIForm->resize(1087, 733);
        tabWidget = new QTabWidget(SIForm);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(70, 30, 961, 691));
        QFont font;
        font.setFamily(QString::fromUtf8("Arial"));
        font.setPointSize(17);
        font.setBold(false);
        font.setItalic(false);
        font.setUnderline(false);
        font.setWeight(50);
        font.setStrikeOut(false);
        tabWidget->setFont(font);
        tabWidget->setAutoFillBackground(true);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        groupBox_DataCollection = new QGroupBox(tab);
        groupBox_DataCollection->setObjectName(QString::fromUtf8("groupBox_DataCollection"));
        groupBox_DataCollection->setGeometry(QRect(20, 10, 330, 100));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Arial"));
        font1.setPointSize(14);
        font1.setBold(false);
        font1.setItalic(false);
        font1.setUnderline(false);
        font1.setWeight(50);
        font1.setStrikeOut(false);
        groupBox_DataCollection->setFont(font1);
        DC_saveButton = new QPushButton(groupBox_DataCollection);
        DC_saveButton->setObjectName(QString::fromUtf8("DC_saveButton"));
        DC_saveButton->setGeometry(QRect(115, 20, 100, 70));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Arial"));
        font2.setPointSize(13);
        font2.setBold(false);
        font2.setItalic(false);
        font2.setUnderline(false);
        font2.setWeight(50);
        font2.setStrikeOut(false);
        DC_saveButton->setFont(font2);
        DC_dictateButton = new QPushButton(groupBox_DataCollection);
        DC_dictateButton->setObjectName(QString::fromUtf8("DC_dictateButton"));
        DC_dictateButton->setGeometry(QRect(220, 20, 100, 70));
        DC_dictateButton->setFont(font2);
        DC_newButton = new QPushButton(groupBox_DataCollection);
        DC_newButton->setObjectName(QString::fromUtf8("DC_newButton"));
        DC_newButton->setGeometry(QRect(10, 20, 100, 70));
        DC_newButton->setFont(font2);
        groupBox_Scale_P = new QGroupBox(tab);
        groupBox_Scale_P->setObjectName(QString::fromUtf8("groupBox_Scale_P"));
        groupBox_Scale_P->setGeometry(QRect(410, 160, 330, 105));
        scalePDecreaseButton = new QPushButton(groupBox_Scale_P);
        scalePDecreaseButton->setObjectName(QString::fromUtf8("scalePDecreaseButton"));
        scalePDecreaseButton->setGeometry(QRect(5, 28, 100, 70));
        scalePIncreaseButton = new QPushButton(groupBox_Scale_P);
        scalePIncreaseButton->setObjectName(QString::fromUtf8("scalePIncreaseButton"));
        scalePIncreaseButton->setGeometry(QRect(225, 28, 100, 70));
        scalePLineEdit = new QLineEdit(groupBox_Scale_P);
        scalePLineEdit->setObjectName(QString::fromUtf8("scalePLineEdit"));
        scalePLineEdit->setGeometry(QRect(110, 33, 110, 61));
        scalePLineEdit->setFocusPolicy(Qt::NoFocus);
        scalePLineEdit->setAlignment(Qt::AlignHCenter);
        scalePLineEdit->setReadOnly(true);
        okButton = new QPushButton(tab);
        okButton->setObjectName(QString::fromUtf8("okButton"));
        okButton->setGeometry(QRect(580, 40, 160, 110));
        okButton->setFont(font1);
        connectionButton = new QPushButton(tab);
        connectionButton->setObjectName(QString::fromUtf8("connectionButton"));
        connectionButton->setGeometry(QRect(10, 110, 190, 100));
        connectionButton->setFont(font1);
        connectionButton->setAutoFillBackground(true);
        connectionButton->setIconSize(QSize(190, 100));
        connectionButton->setFlat(true);
        label_3 = new QLabel(tab);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(213, 130, 171, 23));
        label_3->setFont(font2);
        label_3->setAlignment(Qt::AlignCenter);
        startButton = new QPushButton(tab);
        startButton->setObjectName(QString::fromUtf8("startButton"));
        startButton->setGeometry(QRect(10, 210, 151, 91));
        QFont font3;
        font3.setFamily(QString::fromUtf8("Arial"));
        font3.setPointSize(24);
        font3.setBold(false);
        font3.setItalic(false);
        font3.setUnderline(false);
        font3.setWeight(50);
        font3.setStrikeOut(false);
        startButton->setFont(font3);
        startButton->setIconSize(QSize(390, 280));
        startButton->setFlat(true);
        lineEdit_Time = new QLineEdit(tab);
        lineEdit_Time->setObjectName(QString::fromUtf8("lineEdit_Time"));
        lineEdit_Time->setGeometry(QRect(210, 160, 181, 41));
        lineEdit_Time->setFocusPolicy(Qt::NoFocus);
        lineEdit_Time->setReadOnly(true);
        loginButton = new QPushButton(tab);
        loginButton->setObjectName(QString::fromUtf8("loginButton"));
        loginButton->setGeometry(QRect(410, 40, 160, 110));
        loginButton->setFont(font1);
        logoButton = new QPushButton(tab);
        logoButton->setObjectName(QString::fromUtf8("logoButton"));
        logoButton->setGeometry(QRect(410, 10, 330, 25));
        logoButton->setIconSize(QSize(330, 25));
        logoButton->setFlat(true);
        lcdcurrent = new QLCDNumber(tab);
        lcdcurrent->setObjectName(QString::fromUtf8("lcdcurrent"));
        lcdcurrent->setGeometry(QRect(470, 420, 91, 51));
        lcdnext = new QLCDNumber(tab);
        lcdnext->setObjectName(QString::fromUtf8("lcdnext"));
        lcdnext->setGeometry(QRect(470, 480, 91, 51));
        lcdtrial = new QLCDNumber(tab);
        lcdtrial->setObjectName(QString::fromUtf8("lcdtrial"));
        lcdtrial->setGeometry(QRect(720, 420, 91, 51));
        connectionButton_2 = new QPushButton(tab);
        connectionButton_2->setObjectName(QString::fromUtf8("connectionButton_2"));
        connectionButton_2->setGeometry(QRect(280, 420, 191, 51));
        connectionButton_2->setFont(font1);
        connectionButton_2->setAutoFillBackground(true);
        connectionButton_2->setIconSize(QSize(190, 100));
        connectionButton_2->setFlat(true);
        connectionButton_3 = new QPushButton(tab);
        connectionButton_3->setObjectName(QString::fromUtf8("connectionButton_3"));
        connectionButton_3->setGeometry(QRect(280, 480, 191, 51));
        connectionButton_3->setFont(font1);
        connectionButton_3->setAutoFillBackground(true);
        connectionButton_3->setIconSize(QSize(190, 100));
        connectionButton_3->setFlat(true);
        connectionButton_4 = new QPushButton(tab);
        connectionButton_4->setObjectName(QString::fromUtf8("connectionButton_4"));
        connectionButton_4->setGeometry(QRect(570, 420, 151, 51));
        connectionButton_4->setFont(font1);
        connectionButton_4->setAutoFillBackground(true);
        connectionButton_4->setIconSize(QSize(190, 100));
        connectionButton_4->setFlat(true);
        textexpstatus = new QTextBrowser(tab);
        textexpstatus->setObjectName(QString::fromUtf8("textexpstatus"));
        textexpstatus->setGeometry(QRect(580, 490, 256, 51));
        textgraspstatus = new QTextBrowser(tab);
        textgraspstatus->setObjectName(QString::fromUtf8("textgraspstatus"));
        textgraspstatus->setGeometry(QRect(580, 550, 256, 51));
        groupBox_Scale_G = new QGroupBox(tab);
        groupBox_Scale_G->setObjectName(QString::fromUtf8("groupBox_Scale_G"));
        groupBox_Scale_G->setGeometry(QRect(410, 280, 330, 105));
        scaleGDecreaseButton = new QPushButton(groupBox_Scale_G);
        scaleGDecreaseButton->setObjectName(QString::fromUtf8("scaleGDecreaseButton"));
        scaleGDecreaseButton->setGeometry(QRect(5, 28, 100, 70));
        scaleGIncreaseButton = new QPushButton(groupBox_Scale_G);
        scaleGIncreaseButton->setObjectName(QString::fromUtf8("scaleGIncreaseButton"));
        scaleGIncreaseButton->setGeometry(QRect(225, 28, 100, 70));
        scaleGLineEdit = new QLineEdit(groupBox_Scale_G);
        scaleGLineEdit->setObjectName(QString::fromUtf8("scaleGLineEdit"));
        scaleGLineEdit->setGeometry(QRect(110, 33, 110, 61));
        scaleGLineEdit->setFocusPolicy(Qt::NoFocus);
        scaleGLineEdit->setAlignment(Qt::AlignHCenter);
        scaleGLineEdit->setReadOnly(true);
        groupBox_master = new QGroupBox(tab);
        groupBox_master->setObjectName(QString::fromUtf8("groupBox_master"));
        groupBox_master->setGeometry(QRect(790, 40, 191, 231));
        QFont font4;
        font4.setPointSize(12);
        groupBox_master->setFont(font4);
        checkBox_GRIPPER = new QCheckBox(groupBox_master);
        checkBox_GRIPPER->setObjectName(QString::fromUtf8("checkBox_GRIPPER"));
        checkBox_GRIPPER->setGeometry(QRect(20, 40, 141, 16));
        QFont font5;
        font5.setPointSize(10);
        checkBox_GRIPPER->setFont(font5);
        checkBox_GRIPPER->setChecked(true);
        checkBox_ORI = new QCheckBox(groupBox_master);
        checkBox_ORI->setObjectName(QString::fromUtf8("checkBox_ORI"));
        checkBox_ORI->setGeometry(QRect(20, 60, 161, 16));
        checkBox_ORI->setFont(font5);
        checkBox_ORI->setChecked(true);
        checkBox_POS = new QCheckBox(groupBox_master);
        checkBox_POS->setObjectName(QString::fromUtf8("checkBox_POS"));
        checkBox_POS->setGeometry(QRect(20, 80, 141, 16));
        checkBox_POS->setFont(font5);
        checkBox_POS->setChecked(true);
        checkBox_fullscreen = new QCheckBox(groupBox_master);
        checkBox_fullscreen->setObjectName(QString::fromUtf8("checkBox_fullscreen"));
        checkBox_fullscreen->setGeometry(QRect(20, 140, 141, 16));
        checkBox_fullscreen->setFont(font5);
        checkBox_fullscreen->setChecked(false);
        lcdFPS = new QLCDNumber(tab);
        lcdFPS->setObjectName(QString::fromUtf8("lcdFPS"));
        lcdFPS->setGeometry(QRect(810, 300, 151, 41));
        lcdFPS->setSmallDecimalPoint(false);
        groupBox_5 = new QGroupBox(tab);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setGeometry(QRect(50, 390, 201, 211));
        zeroX = new QDoubleSpinBox(groupBox_5);
        zeroX->setObjectName(QString::fromUtf8("zeroX"));
        zeroX->setGeometry(QRect(40, 60, 121, 21));
        zeroX->setMinimum(-10);
        zeroX->setMaximum(10);
        zeroX->setValue(0);
        zeroY = new QDoubleSpinBox(groupBox_5);
        zeroY->setObjectName(QString::fromUtf8("zeroY"));
        zeroY->setGeometry(QRect(40, 100, 121, 21));
        zeroY->setMinimum(-10);
        zeroY->setMaximum(10);
        zeroZ = new QDoubleSpinBox(groupBox_5);
        zeroZ->setObjectName(QString::fromUtf8("zeroZ"));
        zeroZ->setGeometry(QRect(40, 140, 121, 21));
        zeroZ->setMinimum(-10);
        zeroZ->setMaximum(10);
        label_21 = new QLabel(tab);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setGeometry(QRect(60, 450, 46, 20));
        label_22 = new QLabel(tab);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setGeometry(QRect(60, 490, 46, 20));
        label_23 = new QLabel(tab);
        label_23->setObjectName(QString::fromUtf8("label_23"));
        label_23->setGeometry(QRect(60, 530, 46, 20));
        zerobutton = new QPushButton(tab);
        zerobutton->setObjectName(QString::fromUtf8("zerobutton"));
        zerobutton->setGeometry(QRect(270, 550, 100, 70));
        tabWidget->addTab(tab, QString());
        tab2 = new QWidget();
        tab2->setObjectName(QString::fromUtf8("tab2"));
        label_4 = new QLabel(tab2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 130, 121, 27));
        lCDNumber2 = new QLCDNumber(tab2);
        lCDNumber2->setObjectName(QString::fromUtf8("lCDNumber2"));
        lCDNumber2->setGeometry(QRect(210, 10, 160, 35));
        A_okButton = new QPushButton(tab2);
        A_okButton->setObjectName(QString::fromUtf8("A_okButton"));
        A_okButton->setGeometry(QRect(210, 50, 160, 70));
        A_okButton->setFocusPolicy(Qt::NoFocus);
        A_startButton = new QPushButton(tab2);
        A_startButton->setObjectName(QString::fromUtf8("A_startButton"));
        A_startButton->setGeometry(QRect(10, 10, 191, 110));
        A_startButton->setFocusPolicy(Qt::NoFocus);
        A_startButton->setAutoFillBackground(true);
        A_startButton->setFlat(true);
        cB_IP = new QComboBox(tab2);
        cB_IP->setObjectName(QString::fromUtf8("cB_IP"));
        cB_IP->setGeometry(QRect(10, 160, 351, 51));
        cB_IP->setFont(font1);
        cB_IP->setFocusPolicy(Qt::NoFocus);
        infoText = new QTextEdit(tab2);
        infoText->setObjectName(QString::fromUtf8("infoText"));
        infoText->setGeometry(QRect(380, 10, 351, 261));
        QFont font6;
        font6.setFamily(QString::fromUtf8("Arial"));
        font6.setPointSize(11);
        font6.setBold(false);
        font6.setItalic(false);
        font6.setUnderline(false);
        font6.setWeight(50);
        font6.setStrikeOut(false);
        infoText->setFont(font6);
        infoText->setFocusPolicy(Qt::NoFocus);
        infoText->setReadOnly(true);
        label_5 = new QLabel(tab2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(20, 230, 71, 20));
        QFont font7;
        font7.setFamily(QString::fromUtf8("Arial"));
        font7.setPointSize(14);
        label_5->setFont(font7);
        label_6 = new QLabel(tab2);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(20, 260, 71, 20));
        label_6->setFont(font7);
        label_7 = new QLabel(tab2);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(20, 290, 71, 20));
        label_7->setFont(font7);
        camAngle1 = new QDoubleSpinBox(tab2);
        camAngle1->setObjectName(QString::fromUtf8("camAngle1"));
        camAngle1->setGeometry(QRect(100, 230, 101, 22));
        camAngle1->setMinimum(-180);
        camAngle1->setMaximum(180);
        camAngle2 = new QDoubleSpinBox(tab2);
        camAngle2->setObjectName(QString::fromUtf8("camAngle2"));
        camAngle2->setGeometry(QRect(100, 260, 101, 22));
        camAngle2->setMinimum(-180);
        camAngle2->setMaximum(180);
        camAngle3 = new QDoubleSpinBox(tab2);
        camAngle3->setObjectName(QString::fromUtf8("camAngle3"));
        camAngle3->setGeometry(QRect(100, 290, 101, 22));
        camAngle3->setMinimum(-180);
        camAngle3->setMaximum(180);
        camAngle3->setValue(-45);
        groupBox = new QGroupBox(tab2);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(260, 379, 161, 161));
        camLeftOffY = new QDoubleSpinBox(groupBox);
        camLeftOffY->setObjectName(QString::fromUtf8("camLeftOffY"));
        camLeftOffY->setGeometry(QRect(51, 110, 91, 22));
        camLeftOffY->setMinimum(-1000);
        camLeftOffY->setMaximum(1000);
        camLeftOffX = new QDoubleSpinBox(groupBox);
        camLeftOffX->setObjectName(QString::fromUtf8("camLeftOffX"));
        camLeftOffX->setGeometry(QRect(51, 50, 91, 22));
        camLeftOffX->setMinimum(-1000);
        camLeftOffX->setMaximum(1000);
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 50, 51, 21));
        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(20, 110, 51, 21));
        groupBox_2 = new QGroupBox(tab2);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(430, 380, 161, 161));
        camRightOffX = new QDoubleSpinBox(groupBox_2);
        camRightOffX->setObjectName(QString::fromUtf8("camRightOffX"));
        camRightOffX->setGeometry(QRect(51, 50, 101, 22));
        camRightOffX->setMinimum(-1000);
        camRightOffX->setMaximum(1000);
        camRightOffY = new QDoubleSpinBox(groupBox_2);
        camRightOffY->setObjectName(QString::fromUtf8("camRightOffY"));
        camRightOffY->setGeometry(QRect(51, 110, 101, 22));
        camRightOffY->setMinimum(-1000);
        camRightOffY->setMaximum(1000);
        label_8 = new QLabel(groupBox_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(20, 50, 51, 21));
        label_9 = new QLabel(groupBox_2);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(20, 110, 51, 21));
        label_10 = new QLabel(groupBox_2);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(20, 110, 51, 21));
        groupBox_3 = new QGroupBox(tab2);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(630, 380, 161, 161));
        camZoom = new QDoubleSpinBox(groupBox_3);
        camZoom->setObjectName(QString::fromUtf8("camZoom"));
        camZoom->setGeometry(QRect(51, 50, 91, 22));
        camZoom->setDecimals(0);
        camZoom->setMinimum(0);
        camZoom->setMaximum(5);
        label_2 = new QLabel(groupBox_3);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 50, 51, 21));
        tabWidget->addTab(tab2, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        dialDirection = new QDial(tab_2);
        dialDirection->setObjectName(QString::fromUtf8("dialDirection"));
        dialDirection->setGeometry(QRect(530, 190, 411, 411));
        dialDirection->setMaximum(360);
        dialDirection->setSingleStep(1);
        dialDirection->setSliderPosition(10);
        dialDirection->setOrientation(Qt::Vertical);
        dialDirection->setInvertedAppearance(true);
        dialDirection->setInvertedControls(true);
        dialDirection->setWrapping(true);
        dialDirection->setNotchTarget(1);
        dialDirection->setNotchesVisible(true);
        comboControlMode = new Q3ComboBox(tab_2);
        comboControlMode->setObjectName(QString::fromUtf8("comboControlMode"));
        comboControlMode->setGeometry(QRect(40, 80, 301, 41));
        comboControlMode->setEditable(false);
        label_11 = new QLabel(tab_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(40, 30, 251, 41));
        spinDirection = new QSpinBox(tab_2);
        spinDirection->setObjectName(QString::fromUtf8("spinDirection"));
        spinDirection->setGeometry(QRect(700, 610, 81, 22));
        spinDirection->setButtonSymbols(QAbstractSpinBox::PlusMinus);
        spinDirection->setMaximum(360);
        autoMoveButtonGo = new QPushButton(tab_2);
        autoMoveButtonGo->setObjectName(QString::fromUtf8("autoMoveButtonGo"));
        autoMoveButtonGo->setGeometry(QRect(50, 370, 75, 34));
        autoMoveButtonGo->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 255, 0);"));
        layoutWidget = new QWidget(tab_2);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(330, 200, 121, 151));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        spinAmplitude = new QSpinBox(layoutWidget);
        spinAmplitude->setObjectName(QString::fromUtf8("spinAmplitude"));
        spinAmplitude->setMinimum(1);
        spinAmplitude->setMaximum(200);
        spinAmplitude->setValue(2);

        verticalLayout->addWidget(spinAmplitude);

        spinDuration = new QSpinBox(layoutWidget);
        spinDuration->setObjectName(QString::fromUtf8("spinDuration"));
        spinDuration->setMinimum(1);
        spinDuration->setMaximum(200000);
        spinDuration->setValue(1000);

        verticalLayout->addWidget(spinDuration);

        spinHold = new QSpinBox(layoutWidget);
        spinHold->setObjectName(QString::fromUtf8("spinHold"));
        spinHold->setMaximum(20000);
        spinHold->setValue(1000);

        verticalLayout->addWidget(spinHold);

        spinRepetition = new QSpinBox(layoutWidget);
        spinRepetition->setObjectName(QString::fromUtf8("spinRepetition"));
        spinRepetition->setMinimum(1);
        spinRepetition->setMaximum(300);
        spinRepetition->setValue(2);

        verticalLayout->addWidget(spinRepetition);

        layoutWidget1 = new QWidget(tab_2);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(50, 200, 265, 151));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_13 = new QLabel(layoutWidget1);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        verticalLayout_2->addWidget(label_13);

        label_14 = new QLabel(layoutWidget1);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        verticalLayout_2->addWidget(label_14);

        label_15 = new QLabel(layoutWidget1);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        verticalLayout_2->addWidget(label_15);

        label_16 = new QLabel(layoutWidget1);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        verticalLayout_2->addWidget(label_16);

        autoMoveButtonStop = new QPushButton(tab_2);
        autoMoveButtonStop->setObjectName(QString::fromUtf8("autoMoveButtonStop"));
        autoMoveButtonStop->setGeometry(QRect(50, 420, 181, 91));
        autoMoveButtonStop->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 0, 0);"));
        spinZOffset = new QSpinBox(tab_2);
        spinZOffset->setObjectName(QString::fromUtf8("spinZOffset"));
        spinZOffset->setGeometry(QRect(480, 600, 119, 32));
        spinZOffset->setMinimum(-45);
        spinZOffset->setMaximum(45);
        spinZOffset->setValue(0);
        label_39 = new QLabel(tab_2);
        label_39->setObjectName(QString::fromUtf8("label_39"));
        label_39->setGeometry(QRect(350, 600, 101, 33));
        tabWidget->addTab(tab_2, QString());
        layoutWidget->raise();
        layoutWidget->raise();
        dialDirection->raise();
        comboControlMode->raise();
        label_11->raise();
        spinDirection->raise();
        autoMoveButtonGo->raise();
        autoMoveButtonStop->raise();
        spinZOffset->raise();
        label_39->raise();

        retranslateUi(SIForm);
        QObject::connect(dialDirection, SIGNAL(valueChanged(int)), spinDirection, SLOT(setValue(int)));
        QObject::connect(spinDirection, SIGNAL(valueChanged(int)), dialDirection, SLOT(setValue(int)));
        QObject::connect(autoMoveButtonGo, SIGNAL(clicked()), SIForm, SLOT(open()));

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(SIForm);
    } // setupUi

    void retranslateUi(QDialog *SIForm)
    {
        SIForm->setWindowTitle(QApplication::translate("SIForm", "Surgeon's GUI for a BRL's Laparoscopic Robot System", 0, QApplication::UnicodeUTF8));
        groupBox_DataCollection->setTitle(QApplication::translate("SIForm", "Data Connection", 0, QApplication::UnicodeUTF8));
        DC_saveButton->setText(QApplication::translate("SIForm", "Start", 0, QApplication::UnicodeUTF8));
        DC_dictateButton->setText(QApplication::translate("SIForm", "Dictate Note", 0, QApplication::UnicodeUTF8));
        DC_newButton->setText(QApplication::translate("SIForm", "Generate\n"
"New File", 0, QApplication::UnicodeUTF8));
        groupBox_Scale_P->setTitle(QApplication::translate("SIForm", "Scale Movement", 0, QApplication::UnicodeUTF8));
        scalePDecreaseButton->setText(QApplication::translate("SIForm", "<<", 0, QApplication::UnicodeUTF8));
        scalePIncreaseButton->setText(QApplication::translate("SIForm", ">>", 0, QApplication::UnicodeUTF8));
        okButton->setText(QApplication::translate("SIForm", "CLOSE GUI", 0, QApplication::UnicodeUTF8));
        connectionButton->setText(QApplication::translate("SIForm", "Connection Status", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SIForm", "Elapsed Surgery Time", 0, QApplication::UnicodeUTF8));
        startButton->setText(QApplication::translate("SIForm", "Start", 0, QApplication::UnicodeUTF8));
        loginButton->setText(QApplication::translate("SIForm", "Now Logined\n"
"Press to log-out", 0, QApplication::UnicodeUTF8));
        logoButton->setText(QApplication::translate("SIForm", "bioRobotics Laboratory", 0, QApplication::UnicodeUTF8));
        connectionButton_2->setText(QApplication::translate("SIForm", "Current Object", 0, QApplication::UnicodeUTF8));
        connectionButton_3->setText(QApplication::translate("SIForm", "Next Object", 0, QApplication::UnicodeUTF8));
        connectionButton_4->setText(QApplication::translate("SIForm", "Trial #", 0, QApplication::UnicodeUTF8));
        groupBox_Scale_G->setTitle(QApplication::translate("SIForm", "Scale Gripper", 0, QApplication::UnicodeUTF8));
        scaleGDecreaseButton->setText(QApplication::translate("SIForm", "<<", 0, QApplication::UnicodeUTF8));
        scaleGIncreaseButton->setText(QApplication::translate("SIForm", ">>", 0, QApplication::UnicodeUTF8));
        groupBox_master->setTitle(QApplication::translate("SIForm", "MASTER CONTROL", 0, QApplication::UnicodeUTF8));
        checkBox_GRIPPER->setText(QApplication::translate("SIForm", "ENABLE GRIPPER", 0, QApplication::UnicodeUTF8));
        checkBox_ORI->setText(QApplication::translate("SIForm", "ENABLE ORIENTATION", 0, QApplication::UnicodeUTF8));
        checkBox_POS->setText(QApplication::translate("SIForm", "ENABLE POSITION", 0, QApplication::UnicodeUTF8));
        checkBox_fullscreen->setText(QApplication::translate("SIForm", "FULL SCREEN", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("SIForm", "ZERO POS", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("SIForm", "X", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("SIForm", "Y", 0, QApplication::UnicodeUTF8));
        label_23->setText(QApplication::translate("SIForm", "Z", 0, QApplication::UnicodeUTF8));
        zerobutton->setText(QApplication::translate("SIForm", "ZERO", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("SIForm", "Surgeon", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("SIForm", "IP Address", 0, QApplication::UnicodeUTF8));
        A_okButton->setText(QApplication::translate("SIForm", "CLOSE GUI", 0, QApplication::UnicodeUTF8));
        A_startButton->setText(QApplication::translate("SIForm", "START", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("SIForm", "Theta 1", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("SIForm", "Theta 2", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("SIForm", "Theta 3", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("SIForm", "Left Cam", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SIForm", "X", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("SIForm", "Y", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("SIForm", "Right Cam", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("SIForm", "X", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("SIForm", "Y", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("SIForm", "Y", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("SIForm", "Zoom", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SIForm", "x", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab2), QApplication::translate("SIForm", "Engineer", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("SIForm", "Control Mode", 0, QApplication::UnicodeUTF8));
        autoMoveButtonGo->setText(QApplication::translate("SIForm", "GO", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("SIForm", "Movement Amplitude [mm]", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("SIForm", "Movement Duration [ms]", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("SIForm", "Hold Duration [ms]", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("SIForm", "Repetitions", 0, QApplication::UnicodeUTF8));
        autoMoveButtonStop->setText(QApplication::translate("SIForm", "STOP", 0, QApplication::UnicodeUTF8));
        label_39->setText(QApplication::translate("SIForm", "Z-Offset", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("SIForm", "Page", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SIForm: public Ui_SIForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SIFORM_H
