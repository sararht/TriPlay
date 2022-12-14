#ifndef DEFECTSELECTION_H
#define DEFECTSELECTION_H

#include <QDialog>

namespace Ui {
class defectSelection;
}

class defectSelection : public QDialog
{
    Q_OBJECT

public:
    explicit defectSelection(QWidget *parent = 0);
    ~defectSelection();

    QString currentType() const;
    QString currentPersonalizedType() const;

    double getDepth() const;
    float getDegRot() const;
    QVector<float> getLD() const;
    QVector<float> getWD() const;
    float getTotalLenght() const;
    float getTotalWidth() const;
    float getWidthCrack() const;
    int getNPoints();

    bool isAccepted = false;
    bool isCancelled = false;


private slots:
    void showEvent(QShowEvent *);
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
    void on_type_comboBox_currentIndexChanged(const QString &arg1);
    void on_personalized_comboBox_currentIndexChanged(const QString &arg1);

    void drawCrack();
    void updateWindow();

    void on_totalL_spinBox_valueChanged(double arg1);
    void on_totalW_spinBox_valueChanged(double arg1);
    void on_w_spinBox_valueChanged(double arg1);
    void on_nPoints_crack_spinBox_valueChanged(double arg1);
    void on_toolButton_clicked();
    void on_lineEdit_lengthD_textChanged(const QString &arg1);
    void on_lineEdit_widthD_textChanged(const QString &arg1);

private:
    Ui::defectSelection *ui;
    QVector<float> lD, wD;


};

#endif // DEFECTSELECTION_H
