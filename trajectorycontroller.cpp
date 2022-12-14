#include "trajectorycontroller.h"
#include <qdebug.h>
#include <QVector2D>


QString PrintVector2D(QVector2D v)
{
    return ("(" + QString::number(v.x()) + ", " + QString::number(v.y())+ ")");
}
QString PrintVector3D(QVector3Dd v)
{
    return ("(" + QString::number(v.x()) + ", " + QString::number(v.y()) + ", " + QString::number(v.z())+ ")");
}

TrajectoryController::TrajectoryController()
{
}

void TrajectoryController::WriteXmlPoints(QXmlStreamWriter *xml)
{
    int nPuntos = this->points.length();
    if(nPuntos > 0)
    {
        xml->writeStartElement("Points");
        xml->writeAttribute("nPoints", QString::number(nPuntos));
        for(int k = 0; k < nPuntos; k++)
        {
            xml->writeTextElement("Point", PrintVector3D(this->points.at(k)));
        }
        xml->writeEndElement();
    }
    else
    {
        xml->writeStartElement("Heights");
        xml->writeAttribute("nHeights", QString::number(points.size()));
        for(QPair<double, double> par : this->heights)
       {
           xml->writeTextElement("Height", PrintVector2D(QVector2D(par.first, par.second)));
       }
        xml->writeEndElement();
        xml->writeStartElement("Thetas");
        xml->writeAttribute("nThetas", QString::number(this->thetas.count()));
        for(QPair<double, double> par : this->thetas)
        {
            xml->writeTextElement("Theta", PrintVector2D(QVector2D(par.first, par.second)));
        }
        xml->writeEndElement();

        xml->writeStartElement("Gammas");
        xml->writeAttribute("nGammas", QString::number(this->gammas.count()));
        for(QPair<double, double> par : this->gammas)
        {
            xml->writeTextElement("Gamma", PrintVector2D(QVector2D(par.first, par.second)));
        }
        xml->writeEndElement();
    }
}

void TrajectoryController::WriteXmlInfo(QXmlStreamWriter *xml)
{
    xml->writeStartElement("SurfaceInspectionModule");
    xml->writeAttribute("DateTime", QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
    this->WriteXmlPoints(xml);
    xml->writeEndElement();
}

int TrajectoryController::SaveTrajectory(QString fullpath)
{
    if(fullpath.length() == 0)
            fullpath = this->filename;
        else
            this->filename = fullpath;

    if(fullpath.length() == 0)
    {
        qCritical() << "No valid filename to save module trajectories";
        return -1;
    }

    QFile fileOut(fullpath);
    fileOut.open(QFile::WriteOnly);
    if(!fileOut.isWritable())
    {
        qCritical() << "Open file" << fullpath << "to write trajectory failed";
        return -1;
    }

        QXmlStreamWriter xmlFileOut(&fileOut);
        xmlFileOut.setAutoFormatting(true);
        xmlFileOut.setAutoFormattingIndent(-1);

        xmlFileOut.writeStartDocument();
        this->WriteXmlInfo(&xmlFileOut);
        xmlFileOut.writeEndDocument();
        fileOut.close();

        return 0;
}

QString TrajectoryController::FindNextStartTag(QXmlStreamReader *xml, QString tag)
{
    QString tagName;
    xml->readNext();    //Saltamos la actual y avanzamos.
    while(!xml->atEnd())
    {
        QXmlStreamReader::TokenType type = xml->tokenType();
        if(tag.length() && type == QXmlStreamReader::EndElement && xml->name().toString().compare(tag, Qt::CaseInsensitive) == 0)
            return "";
        if(type != QXmlStreamReader::StartElement)
        {
            xml->readNext();
            continue;
        }
        tagName = xml->name().toString();
        break;
    }
    return tagName;
}

qint32 TrajectoryController::ReadAttribute(QXmlStreamAttributes *attrs, QString attrname, qint32 *out, qint32 defaultvalue)
{
    qint32 internal = 0.0;
    QStringRef attr = attrs->value(attrname);
    if(!attr.isNull())
        internal = attr.toInt();
    else
        internal = defaultvalue;
    if(out != nullptr)
        *out = internal;
    //LogManager::factoryLogManager()->EscribirTexto(TR("Attribute").toStdString()+COMILLAS_INICIO+attrname.toStdString()+COMILLAS_FIN+IGUAL+COMILLAS_INICIO+QString("%1").arg(internal).toStdString()+COMILLAS_FIN, LogManager::log_log5, false);
    return internal;
}

QPair<double, double> TrajectoryController::ParsePair(QString str)
{
    str = str.remove("(").remove(")").remove("[").remove("]").remove("{").remove("}");
    QStringList campos = str.split(",");
    QPair<double, double> pair;
    if(campos.length() > 0)
        pair.first = campos.at(0).toDouble();
    if(campos.length() > 1)
        pair.second = campos.at(1).toDouble();
    return pair;
}

int TrajectoryController::LoadTrajectory(const QString &fullpath)
{
    QXmlStreamReader xmlFileIn;
    this->points.clear();
    this->filename = fullpath;
    QFile fileIn(fullpath);
    if(!fileIn.exists())
    {
        qCritical() << "Reading file" << fullpath << ". File doesn't exists.";
        return -1;
    }
    fileIn.open(QIODevice::ReadOnly | QFile::Text);
    xmlFileIn.setDevice(&fileIn);
    if(xmlFileIn.error() != QXmlStreamReader::NoError)
    {
        qCritical() << "Reading file" << fullpath << ", XML parse error: " << xmlFileIn.errorString() <<", line: " << xmlFileIn.lineNumber() << ", column: " << xmlFileIn.columnNumber();
        fileIn.close();
        return -2;
    }
    int nTrajectoryPoints = 0;
    int nPointsReaded = 0;
    // Miguel: modificamos para que pueda cargar trayectoria de 3 listas independientes en un lugar de 1 sola
    int nHeightPoints = 0;
    int nThetaPoints = 0;
    int nGammaPoints = 0;
    int nHeightsReaded = 0;
    int nThetasReaded = 0;
    int nGammasReaded = 0;
    while(!xmlFileIn.atEnd())
    {
        QString tagName = this->FindNextStartTag(&xmlFileIn);
        tagName = xmlFileIn.name().toString();
        if(tagName.length() == 0)
            continue;
        //Cuando se llama a un metodo externo de procesamiento este devuelve el xml en el siguiente elemento, no es necesario incrementarlo aqui.
        if(tagName.compare("SurfaceInspectionModule", Qt::CaseInsensitive) == 0)
        {
        }

        else if(tagName.compare("Points",Qt::CaseInsensitive) == 0)
        {
            QXmlStreamAttributes attrs = xmlFileIn.attributes();
            nTrajectoryPoints = this->ReadAttribute(&attrs, QString("nPoints"), nullptr, 0);
            this->points.resize(nTrajectoryPoints);
        }
        /*
        else if(tagName.compare("Points",Qt::CaseInsensitive) == 0)
        {
            if(nPointsReaded >= nTrajectoryPoints)
            {
                qCritical() << "Found more trajectory points than defined";
                continue;
            }

            QString data = xmlFileIn.readElementText();
            QVector4D v = ParseVector4D(data);
            this->puntosTraj[nPointsReaded] = v;
            nPointsReaded++;
        }
        */
        else if (tagName.compare("Heights", Qt::CaseInsensitive) == 0)
        {
            QXmlStreamAttributes attrs = xmlFileIn.attributes();
            nHeightPoints = this->ReadAttribute(&attrs, QString("nHeights"), nullptr, 0);
            this->heights.resize(nHeightPoints);
        }
        else if (tagName.compare("Height", Qt::CaseInsensitive) == 0)
        {
            if (nHeightsReaded >= nHeightPoints)
            {
                qCritical() << "Found more height points than defined";
                continue;
            }
            const QString data = xmlFileIn.readElementText();
            const QPair<double, double> v = ParsePair(data);
            this->heights[nHeightsReaded] = v;
            nHeightsReaded++;
        }

        else if (tagName.compare("Thetas", Qt::CaseInsensitive) == 0)
        {
            QXmlStreamAttributes attrs = xmlFileIn.attributes();
            nThetaPoints = this->ReadAttribute(&attrs, QString("nThetas"), nullptr, 0);
            this->thetas.resize(nThetaPoints);
        }
        else if (tagName.compare("Theta", Qt::CaseInsensitive) == 0)
        {
            if (nThetasReaded >= nThetaPoints)
            {
                qCritical() << "Found more theta points than defined";
                continue;
            }
            const QString data = xmlFileIn.readElementText();
            const QPair<double, double> v = ParsePair(data);
            this->thetas[nThetasReaded] = v;
            nThetasReaded++;
        }
        /*
        else if (tagName.compare(TAG_THETA_DEG, Qt::CaseInsensitive) == 0)
        {
            if (nThetasReaded >= nThetaPoints)
            {
                DSI_LOG_WARNING(tr("Found more theta points than defined"));
                continue;
            }
            const QString data = xmlFileIn.readElementText();
            QPair<double, double> v = ParsePair(data);
            v.first = v.first;
            v.second = DEGREES_TO_RADIANS(v.second);
            this->thetasTraj[nThetasReaded] = v;
            nThetasReaded++;

        }
        */
        else if (tagName.compare("Gammas", Qt::CaseInsensitive) == 0)
        {
            QXmlStreamAttributes attrs = xmlFileIn.attributes();
            nGammaPoints = this->ReadAttribute(&attrs, QString("nGammas"), nullptr, 0);
            this->gammas.resize(nGammaPoints);
        }
        else if (tagName.compare("Gamma", Qt::CaseInsensitive) == 0)
        {
            if (nGammasReaded >= nGammaPoints)
            {
                qCritical() << "Found more gamma points than defined";
                continue;
            }
            const QString data = xmlFileIn.readElementText();
            const QPair<double, double> v = ParsePair(data);
            this->gammas[nGammasReaded] = v;
            nGammasReaded++;
        }
        /*
        else if (tagName.compare(TAG_GAMMA_DEG, Qt::CaseInsensitive) == 0)
        {
            if (nGammasReaded >= nGammaPoints)
            {
                DSI_LOG_WARNING(tr("Found more gamma points than defined"));
                continue;
            }
            const QString data = xmlFileIn.readElementText();
            QPair<double, double> v = ParsePair(data);
            v.first = v.first;
            v.second = DEGREES_TO_RADIANS(v.second);
            this->gammasTraj[nGammasReaded] = v;
            nGammasReaded++;
        }
*/
    }
    fileIn.close();

//    for (const auto &point : this->puntosTraj) {
//        qDebug() << point;
//    }

    return 0;
}
