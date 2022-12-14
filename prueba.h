#ifndef PRUEBA_H
#define PRUEBA_H

#include <QObject>

class prueba : public QObject
{
    Q_OBJECT
public:
    explicit prueba(QObject *parent = nullptr);

signals:

};

#endif // PRUEBA_H
