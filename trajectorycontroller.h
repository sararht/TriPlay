#ifndef TRAJECTORYCONTROLLER_H
#define TRAJECTORYCONTROLLER_H

#include <QVector>
#include <qvector3dd.h>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFile>
#include <QTime>

class TrajectoryController
{
public:
    TrajectoryController();
    int SaveTrajectory(QString fullpath = "");
    int LoadTrajectory(const QString &fullpath);
    void set3DPoints(QVector<QVector3Dd> p){points=p;};
    void setRPY(QVector<QVector3Dd> rot){rpy=rot;};


private:
    QVector<QVector3Dd> points;
    QVector<QVector3Dd> rpy;

    QVector<QPair<double, double>> heights, thetas, gammas;

    QString filename;


    void WriteXmlInfo(QXmlStreamWriter *xml);
    void WriteXmlPoints(QXmlStreamWriter *xml);
    QString FindNextStartTag(QXmlStreamReader *xml, QString tag="");
    qint32 ReadAttribute(QXmlStreamAttributes *attrs, QString attrname, qint32 *out, qint32 defaultvalue);
    QPair<double, double> ParsePair(QString str);



};

#endif // TRAJECTORYCONTROLLER_H
