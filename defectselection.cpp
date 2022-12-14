#include "defectselection.h"
#include "ui_defectselection.h"
#include <sstream>
#include <stdlib.h>
#include <QGraphicsPixmapItem>
#include <iostream>
#include <QFileDialog>

defectSelection::defectSelection(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::defectSelection)
{
    ui->setupUi(this);
}

defectSelection::~defectSelection()
{
    delete ui;
}

void defectSelection::showEvent(QShowEvent *)
{
    updateWindow();
    on_type_comboBox_currentIndexChanged(ui->type_comboBox->currentText());
}

QString defectSelection::currentType() const
{
    return ui->type_comboBox->currentText();
}
QString defectSelection::currentPersonalizedType() const
{
    return ui->personalized_comboBox->currentText();
}

double defectSelection::getDepth() const
{
    return ui->depth_spinBox->value();
}

float defectSelection::getDegRot() const
{
    return ui->degRot_spinBox->value();
}

void defectSelection::on_buttonBox_accepted()
{
    this->isAccepted = true;
}

void defectSelection::on_buttonBox_rejected()
{
    this->isCancelled = true;
}

QVector<float> defectSelection::getLD() const
{
    return this->lD;
}

QVector<float> defectSelection::getWD() const
{
    return this->wD;
}


float defectSelection::getWidthCrack() const
{
    return ui->w_spinBox->value();
}

int defectSelection::getNPoints()
{
    std::cout << "PARAMSSS: " <<ui->nPoints_crack_spinBox->value() << std::endl;

    return ui->nPoints_crack_spinBox->value();
}

void defectSelection::updateWindow()
{
    QGraphicsScene* scene = new QGraphicsScene();
    QGraphicsPixmapItem* item;

    if(ui->type_comboBox->currentText() == "Personalized")
    {

        std::stringstream name_image;
        name_image << ":/MyRes/defects_catalog/images/" << ui->personalized_comboBox->currentText().toStdString() << ".jpg";
        item = new QGraphicsPixmapItem(QPixmap(QString::fromStdString(name_image.str())));

    }

    else
    {
        std::stringstream name_image;
        name_image << ":/MyRes/defects_catalog/images/" << ui->type_comboBox->currentText().toStdString() << ".jpg";
        item = new QGraphicsPixmapItem(QPixmap(QString::fromStdString(name_image.str())));
    }
    ui->graphicsView->setScene(scene);
    ui->graphicsView->fitInView(item,Qt::KeepAspectRatio);
    scene->addItem(item);
    ui->graphicsView->show();

    if(ui->type_comboBox->currentText() == "Crack")
        this->on_nPoints_crack_spinBox_valueChanged(this->ui->nPoints_crack_spinBox->value());
}

void defectSelection::on_type_comboBox_currentIndexChanged(const QString &arg1)
{
    if(arg1 == "Personalized")
    {
        ui->personalized_comboBox->setVisible(true);
        ui->label_3->setVisible(true);
        ui->personalized_comboBox->setEnabled(true);
        ui->label_3->setEnabled(true);
        ui->toolButton->setVisible(true);
    }
    else
    {
        ui->personalized_comboBox->setVisible(false);
        ui->label_3->setVisible(false);
        ui->personalized_comboBox->setEnabled(false);
        ui->label_3->setEnabled(false);
        ui->toolButton->setVisible(false);

    }

    if(arg1 == "Crack")
    {
        ui->crackWidget->show();
    }
    else
    {
        ui->crackWidget->hide();
    }

    updateWindow();
}

void defectSelection::on_personalized_comboBox_currentIndexChanged(const QString &arg1)
{
    updateWindow();
}

void defectSelection::drawCrack()
{
    float w_max = 1800;
    float l_max = 1000;

    QGraphicsScene* scene = new QGraphicsScene();
    QGraphicsRectItem* item1 = new QGraphicsRectItem(0,0,w_max,l_max);


    float l_prev = 0;
    float w_prev = l_max*wD[0];
    QGraphicsEllipseItem* point0 = new QGraphicsEllipseItem(l_prev,w_prev,20,20);
    point0->setBrush(QBrush(Qt::red));
    scene->addItem(point0);

    for (int i=1; i<ui->nPoints_crack_spinBox->value()+2; i++)
    {
        float l_i = w_max*lD[i];
        float w_i = l_max*wD[i];

        QGraphicsLineItem* line_i = new QGraphicsLineItem(l_prev, w_prev, l_i, w_i);
        QGraphicsEllipseItem* point_i = new QGraphicsEllipseItem(l_i,w_i,20,20);
        point_i->setBrush(QBrush(Qt::red));
        scene->addItem(point_i);
        scene->addItem(line_i);
        l_prev = l_i;
        w_prev = w_i;
    }

    ui->graphicsView->setScene(scene);
    scene->addItem(item1);
    ui->graphicsView->show();
}


void defectSelection::on_totalL_spinBox_valueChanged(double arg1)
{
    drawCrack();
}

void defectSelection::on_totalW_spinBox_valueChanged(double arg1)
{
    drawCrack();
}

void defectSelection::on_w_spinBox_valueChanged(double arg1)
{
    drawCrack();
}

void defectSelection::on_nPoints_crack_spinBox_valueChanged(double arg1)
{
    lD.clear();
    wD.clear();

    float v = 1/(arg1+1);
    float v2 = ((int)(v * 100 + .5) / 100.0);
    std::stringstream lengthD_text, widthD_text;
    lengthD_text << 0 << ", ";
    widthD_text << 0.5 << ", ";
    lD.push_back(0);
    wD.push_back(0.5);

    for (int i=0; i<arg1; i++)
    {
       lengthD_text << v2+v2*i << ", ";
       widthD_text << 0.5 << ", ";
       lD.push_back(v+v*i);
       wD.push_back(0.5);
    }
    lengthD_text << 1;
    widthD_text << 0.5;
    lD.push_back(1);
    wD.push_back(0.5);

    ui->lineEdit_lengthD->setText(QString(lengthD_text.str().c_str()));
    ui->lineEdit_widthD->setText(QString(widthD_text.str().c_str()));


    drawCrack();

}


void defectSelection::on_toolButton_clicked()
{
    //Manage defect catalogue
    QString file_name = QFileDialog::getOpenFileName(this, "Add new personalized defect", QDir::homePath(),"XML Files (*.xml)");

    //Copiar xml en el directorio correcto
    QFile file(file_name);
    std::stringstream new_file;
    QFileInfo fileInfo(file.fileName());
    QString name = fileInfo.fileName();
    new_file << "../simulador/defects_catalog/" << name.toStdString();

    file.copy(QString::fromStdString(new_file.str()));
    file.close();

    //Actualizar el desplegable de los defectos -> Hacer que siempre lea lo que hay en el directorio de catalogue_defects o algo
    QStringList name_;
    name_ = name.split(".");
    ui->personalized_comboBox->addItem(name_[0]);

    //Insertar imagen para la previsualización
    QString image_name = QFileDialog::getOpenFileName(this, "Add image for previsualization", QDir::homePath(),"JPG Files (*.jpg)");


}

void defectSelection::on_lineEdit_lengthD_textChanged(const QString &arg1)
{
    QString arg = arg1;
    arg.replace(" ", "");
    arg.replace(",", " ");
    QStringList list = arg.split(' ');

    lD.clear();
    for (int i=0; i<list.size(); i++)
    {
        QString numberGroup = list.at(i);
        lD.push_back(numberGroup.toFloat());
    }

    drawCrack();
}

void defectSelection::on_lineEdit_widthD_textChanged(const QString &arg1)
{
    QString arg = arg1;
    arg.replace(" ", "");
    arg.replace(",", " ");
    QStringList list = arg.split(' ');

    wD.clear();
    for (int i=0; i<list.size(); i++)
    {
        QString numberGroup = list.at(i);
        wD.push_back(numberGroup.toFloat());
    }

    drawCrack();
}
