#include "basicplot.h"
#include "ui_basicplot.h"

basicPlot::basicPlot(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::basicPlot)
{
    ui->setupUi(this);
}

basicPlot::~basicPlot()
{
    delete ui;
}

void basicPlot::plot(QVector<QVector3Dd> &data)
{
    QVector<float> data_z;

    for (int i=0; i<data.size();i++)
        data_z.push_back(data[i].z());

    QVector<double> data_aux(data_z.begin(), data_z.end());
    QVector<double> keys;

    for(int i=0; i<data_z.size();i++)
        keys.push_back(i);


    ui->widget->addGraph();
    ui->widget->graph(0)->setData(keys, data_aux);

    ui->widget->xAxis->setLabel("x");
    ui->widget->yAxis->setLabel("y");
    ui->widget->graph(0)->setPen(QPen(Qt::blue));
    ui->widget->xAxis->setRange(0, data_z.size());
    float max = *std::max_element(data_z.begin(), data_z.end());
    float min = *std::min_element(data_z.begin(), data_z.end());
    ui->widget->yAxis->setRange(min,max);
    // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
    ui->widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->widget->replot();


}

