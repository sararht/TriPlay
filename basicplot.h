#ifndef BASICPLOT_H
#define BASICPLOT_H

#include <QWidget>
#include "qvector3dd.h"

namespace Ui {
class basicPlot;
}

class basicPlot : public QWidget
{
    Q_OBJECT

public:
    explicit basicPlot(QWidget *parent = nullptr);
    ~basicPlot();
    void plot(QVector<QVector3Dd> &data);

private:
    Ui::basicPlot *ui;
};

#endif // BASICPLOT_H
