#include "render.h"

QMatrix3x3 multiplyMatrix3x3(QMatrix3x3 mat1, QMatrix3x3 mat2)
{
    return mat1*mat2;
}

void  myRenderer::picker_Clicked(Qt3DRender::QPickEvent *pick)
{
  std::cout << "picker_Clicked" << std::endl;
}

void myRenderer::changeBackgroundColor(QColor color)
{
    view->defaultFrameGraph()->setClearColor(color);
}

myRenderer::myRenderer(sensor sensor_model, QString name_file_)
{

    //sensor_model = sensor_;
    view = new Qt3DExtras::Qt3DWindow;
    view->defaultFrameGraph()->setClearColor(QColor(82,87,110));

    view_axis = new Qt3DExtras::Qt3DWindow();

    rootEntity = new Qt3DCore::QEntity;

    //STL MODEL
    modelEntity = new Qt3DCore::QEntity(rootEntity);
    modelMesh = new Qt3DRender::QMesh;
    modelMesh->setSource(QUrl::fromLocalFile(name_file_));
    modelMesh->setMeshName("model");

    Qt3DExtras::QPhongMaterial *modelMaterial = new Qt3DExtras::QPhongMaterial;
    modelMaterial->setDiffuse(QColor(254,254,254));

    modelEntity->addComponent(modelMaterial);
    modelEntity->addComponent(modelMesh);

    //LASER
    float FOV_mm = sensor_model.range*tan(sensor_model.FOV/2);
    laserEntity = new Qt3DCore::QEntity(rootEntity);
    laserMesh = new Qt3DExtras::QConeMesh;
    laserMesh->setTopRadius(0);
    laserMesh->setBottomRadius(FOV_mm);
    laserMesh->setLength(sensor_model.range);
    laserMesh->setSlices(2);

    Qt3DExtras::QPhongMaterial *laserMaterial = new Qt3DExtras::QPhongMaterial;
    laserMaterial->setDiffuse(QColor(254, 50, 50));
    laserMaterial->setAmbient(QColor(254, 50, 50));
    laserMaterial->setSpecular(QColor(254, 50, 50));
    laserMaterial->setShininess(0.8);

    laserTransform = new Qt3DCore::QTransform;

    laserEntity->addComponent(laserMesh);
    laserEntity->addComponent(laserMaterial);
    laserEntity->addComponent(laserTransform);

    //SENSOR
    sensorEntity = new Qt3DCore::QEntity(rootEntity);
    sensorMesh = new Qt3DExtras::QCuboidMesh;
    sensorMesh->setXExtent(20);
    sensorMesh->setYExtent(130);
    sensorMesh->setZExtent(75);

    sensorTransform = new Qt3DCore::QTransform;

    sensorEntity->addComponent(sensorMesh);
    sensorEntity->addComponent(modelMaterial);
    sensorEntity->addComponent(sensorTransform);

    //CAMERA
    camera = view->camera();
    camera->lens()->setPerspectiveProjection(45.0, 16.0/9.0, 0.1, 10000.0);
    camera->setViewCenter(sensor_model.origin.toQVector3D());
    camera->setPosition(QVector3D(sensor_model.origin.x(),sensor_model.origin.y()+1000,sensor_model.origin.z()-1000));

    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(rootEntity);
    camController->setLinearSpeed(2000);
    camController->setLookSpeed(1000);
    camController->setCamera(camera);


    //LIGHT
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity);
    light->setColor("white");
    light->setIntensity(0.7);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform;
    lightTransform->setTranslation(QVector3D(0, 0, 1000.0));

    lightEntity->addComponent(light);
    lightEntity->addComponent(lightTransform);

    Qt3DCore::QEntity *lightEntity2 = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light2 = new Qt3DRender::QPointLight(lightEntity2);
    light2->setColor("white");
    light2->setIntensity(0.7);
    Qt3DCore::QTransform *lightTransform2 = new Qt3DCore::QTransform;
    lightTransform2->setTranslation(QVector3D(500, -1000, -1000.0));

    lightEntity2->addComponent(light2);
    lightEntity2->addComponent(lightTransform2);

    Qt3DCore::QEntity *lightEntity3 = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light3 = new Qt3DRender::QPointLight(lightEntity3);
    light3->setColor("white");
    light3->setIntensity(0.5);
    Qt3DCore::QTransform *lightTransform3 = new Qt3DCore::QTransform;
    lightTransform3->setTranslation(QVector3D(-3000, 10000, 4000));

    lightEntity3->addComponent(light3);
    lightEntity3->addComponent(lightTransform3);

    //CONTAINER
    view->setRootEntity(rootEntity);

    QSize screenSize_stl = view->screen()->size();
    container_stl = QWidget::createWindowContainer(view);
    container_stl->setMinimumSize(QSize(screenSize_stl.width() / 2, screenSize_stl.height() / 2));
    container_stl->setMaximumSize(screenSize_stl);
    container_stl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container_stl->setFocusPolicy(Qt::StrongFocus);


}

void myRenderer::setRotation0(Qt3DCore::QEntity *entity)
{
    Qt3DCore::QTransform *transform = new Qt3DCore::QTransform;
    QQuaternion rot0 = QQuaternion::fromEulerAngles(0,0,0);;
    transform->setRotation(rot0);
    entity->addComponent(transform);
}

inline QVector3Dd ToEulerAngles(QQuaternion &q)
{
    QVector3Dd angles;

    q.normalize();
    double test = q.x()*q.y() + q.z()*q.scalar();
    double roll, pitch, yaw;

    if (test > 0.499) { // singularity at north pole
            pitch = 2 * atan2(q.x(),q.scalar());
            yaw = PI/2;
            roll = 0;
        }
    else if (test < -0.499) { // singularity at south pole
        pitch = -2 * atan2(q.x(),q.scalar());
        yaw = -PI/2;
        roll = 0;
    }

    double sqx = q.x()*q.x();
    double sqy = q.y()*q.y();
    double sqz = q.z()*q.z();

    pitch = atan2(2*q.y()*q.scalar()-2*q.x()*q.z() , 1 - 2*sqy - 2*sqz);
    yaw = asin(2*test);
    roll = atan2(2*q.x()*q.scalar()-2*q.y()*q.z() , 1 - 2*sqx - 2*sqz);




/*
    double x = q.x();
    double y=q.y();
    double z=q.z();
    double scalar = q.scalar();

   QVector3Dd angles;
   double pitch, yaw, roll;
   double sinr_cosp = 2 * (q.scalar()* q.x() + q.y() * q.z());
   double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
   roll = std::atan2(sinr_cosp, cosr_cosp);

   // pitch (y-axis rotation)
   double sinp = 2 * (q.scalar() * q.y()- q.z() * q.x());
   if (std::abs(sinp) >= 1)
       pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
   else
       pitch = std::asin(sinp);

   // yaw (z-axis rotation)
   double siny_cosp = 2 * (q.scalar() * q.z() + q.x() * q.y());
   double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
   yaw = std::atan2(siny_cosp, cosy_cosp);

*/
   angles.setX(roll);
   angles.setY(pitch);
   angles.setZ(yaw);

   angles = angles*180/PI;
       return angles;
}

void myRenderer::freeMemory()
{
    //free memory
    delete modelMesh;
//    delete laserMesh;
//    delete sensorMesh;
//    delete sensorTransform;
//    delete laserTransform;

    modelMesh = new Qt3DRender::QMesh;
//    laserMesh = new Qt3DExtras::QConeMesh;;
//    sensorMesh = new Qt3DExtras::QCuboidMesh;;
//    sensorTransform = new Qt3DCore::QTransform;
//    laserTransform = new Qt3DCore::QTransform;

}
void myRenderer::updateRenderer(sensor sensor_model)
{

   // freeMemory();

  //  setRotation0(sensorEntity);
  //  setRotation0(laserEntity);


    //LASER
    float values[9] = {-1,0,0,0,-1,0,0,0,1};
    QMatrix3x3 mat_laserW(values);

    QQuaternion quat_laserW_ = QQuaternion::fromEulerAngles(-90,0,0);
    QMatrix3x3 mat_laserW_ = quat_laserW_.toRotationMatrix();

    float x = sensor_model.range*tan(sensor_model.FOV/2);
    laserMesh->setBottomRadius(x);
    laserMesh->setLength(sensor_model.range);

    //QQuaternion quaternion = QQuaternion::fromEulerAngles(sensor_model.pitch,sensor_model.yaw,sensor_model.roll);

    QQuaternion quaternion = sensor_model.q;
    QMatrix3x3 mat_aux = quaternion.toRotationMatrix();
    QMatrix3x3 final_rotM = multiplyMatrix3x3(mat_aux,mat_laserW);
    final_rotM = multiplyMatrix3x3(final_rotM,mat_laserW_);
    QQuaternion final_rot = QQuaternion::fromRotationMatrix(final_rotM);

    laserTransform->setRotation(final_rot); //final_rot-------------

    QVector3D t(0,-sensor_model.range/2,0);
    QVector3D t_ = sensor_model.origin.toQVector3D()+final_rot.rotatedVector(t);
    laserTransform->setTranslation(t_);

    laserEntity->addComponent(laserTransform);
    laserEntity->addComponent(laserMesh);

    //SENSOR
    QVector3D t_s = sensor_model.origin.toQVector3D() + quaternion.rotatedVector(QVector3D(0,0,-30));
    sensorTransform->setRotation(quaternion);
    sensorEntity->addComponent(sensorTransform);

    sensorTransform->setTranslation(t_s);

    sensorEntity->addComponent(sensorTransform);

}

void myRenderer::changeModel(QString name_file, sensor sensor_model)
{

    freeMemory(); //CAMBIO

    modelMesh->setSource(QUrl::fromLocalFile(name_file));
    modelEntity->addComponent(modelMesh);

    camera->setViewCenter(sensor_model.origin.toQVector3D());
    camera->setPosition(QVector3D(sensor_model.origin.x(),sensor_model.origin.y()+1000,sensor_model.origin.z()-500));

}

Qt3DCore::QEntity *drawDefect(QVector<QVector3D>pos, Qt3DCore::QEntity *_rootEntity)
{
    QColor color(0,0,255);

    auto *geometry = new Qt3DRender::QGeometry(_rootEntity);

    // Points
    QByteArray bufferBytes;
    bufferBytes.resize(3*pos.size()* sizeof(float));
    float *positions = reinterpret_cast<float*>(bufferBytes.data());

    QByteArray indexBytes;
    indexBytes.resize(pos.size() * sizeof(unsigned int)); // start to end
    unsigned int *indices = reinterpret_cast<unsigned int*>(indexBytes.data());

    for (int i=0; i<pos.size();i++)
    {
        positions[3*i] = pos[i].x();
        positions[3*i+1] = pos[i].y();
        positions[3*i+2] = pos[i].z();

        int ind = 0;
        if (3*i%3==0)
            ind++;

        indices[i] = i;


    }
    auto *buf = new Qt3DRender::QBuffer(geometry);
    buf->setData(bufferBytes);

    auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute); /////////////
    positionAttribute->setBuffer(buf);
    positionAttribute->setByteStride(3* sizeof(float));
    positionAttribute->setCount(pos.size());
    geometry->addAttribute(positionAttribute); // We add the vertices in the geometry


    auto *indexBuffer = new Qt3DRender::QBuffer(geometry);
    indexBuffer->setData(indexBytes);

    auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
    indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(pos.size());
    geometry->addAttribute(indexAttribute); // We add the indices linking the points in the geometry

    // Mesh
    auto *defect = new Qt3DRender::QGeometryRenderer(_rootEntity);
    defect->setGeometry(geometry);
    defect->setPrimitiveType(Qt3DRender::QGeometryRenderer::Points); //TriangleStrip, TriangleStripAdjancency

    auto *material = new Qt3DExtras::QPhongMaterial(_rootEntity);
    material->setDiffuse(QColor(254,0,0));

    // Entity
    auto *defectEntity = new Qt3DCore::QEntity(_rootEntity);
    defectEntity->addComponent(defect);
    defectEntity->addComponent(material);

    return defectEntity;
}

Qt3DCore::QEntity *drawTraj(QVector<QVector3Dd>pos, Qt3DCore::QEntity *_rootEntity, QColor color)
{
    auto *geometry = new Qt3DRender::QGeometry(_rootEntity);

    // Points
    QByteArray bufferBytes;
    bufferBytes.resize(3*pos.size()* sizeof(float));
    float *positions = reinterpret_cast<float*>(bufferBytes.data());

    QByteArray indexBytes;
    indexBytes.resize(pos.size() * sizeof(unsigned int)); // start to end
    unsigned int *indices = reinterpret_cast<unsigned int*>(indexBytes.data());

    for (int i=0; i<pos.size();i++)
    {
        positions[3*i] = pos[i].x();
        positions[3*i+1] = pos[i].y();
        positions[3*i+2] = pos[i].z();

        int ind = 0;
        if (3*i%3==0)
            ind++;

        indices[i] = i;
    }
    auto *buf = new Qt3DRender::QBuffer(geometry);
    buf->setData(bufferBytes);

    auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute); /////////////
    positionAttribute->setBuffer(buf);
    positionAttribute->setByteStride(3* sizeof(float));
    positionAttribute->setCount(pos.size());
    geometry->addAttribute(positionAttribute); // We add the vertices in the geometry


    auto *indexBuffer = new Qt3DRender::QBuffer(geometry);
    indexBuffer->setData(indexBytes);

    auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
    indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(pos.size());
    geometry->addAttribute(indexAttribute); // We add the indices linking the points in the geometry

    // Mesh
    auto *defect = new Qt3DRender::QGeometryRenderer(_rootEntity);
    defect->setGeometry(geometry);
    defect->setPrimitiveType(Qt3DRender::QGeometryRenderer::Points); //TriangleStrip, TriangleStripAdjancency

    auto *material = new Qt3DExtras::QPhongMaterial(_rootEntity);
    material->setDiffuse(color);
    material->setAmbient(color);

    // Entity
    auto *trajEntity = new Qt3DCore::QEntity(_rootEntity);
    trajEntity->addComponent(defect);
    trajEntity->addComponent(material);

    return trajEntity;
}

Qt3DCore::QEntity *drawLine(const QVector3D& start, const QVector3D& end, const QColor& color, Qt3DCore::QEntity *_rootEntity)
{
    QColor color_(0,0,0);

    auto *geometry = new Qt3DRender::QGeometry(_rootEntity);

    // position vertices (start and end)
    QByteArray bufferBytes;
    bufferBytes.resize(3 * 2 * sizeof(float)); // start.x, start.y, start.end + end.x, end.y, end.z
    float *positions = reinterpret_cast<float*>(bufferBytes.data());
    *positions++ = start.x();
    *positions++ = start.y();
    *positions++ = start.z();
    *positions++ = end.x();
    *positions++ = end.y();
    *positions++ = end.z();

    auto *buf = new Qt3DRender::QBuffer(geometry);
    buf->setData(bufferBytes);

    auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(buf);
    positionAttribute->setByteStride(3 * sizeof(float));
    positionAttribute->setCount(2);
    geometry->addAttribute(positionAttribute); // We add the vertices in the geometry

    // connectivity between vertices
    QByteArray indexBytes;
    indexBytes.resize(2 * sizeof(unsigned int)); // start to end
    unsigned int *indices = reinterpret_cast<unsigned int*>(indexBytes.data());
    *indices++ = 0;
    *indices++ = 1;

    auto *indexBuffer = new Qt3DRender::QBuffer(geometry);
    indexBuffer->setData(indexBytes);

    auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
    indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(2);
    geometry->addAttribute(indexAttribute); // We add the indices linking the points in the geometry

    // mesh
    auto *line = new Qt3DRender::QGeometryRenderer(_rootEntity);
    line->setGeometry(geometry);
    line->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    auto *material = new Qt3DExtras::QPhongMaterial(_rootEntity);
    material->setAmbient(color);

    // entity
    auto *lineEntity = new Qt3DCore::QEntity(_rootEntity);
    lineEntity->addComponent(line);
    lineEntity->addComponent(material);

    return lineEntity;
}

void myRenderer::addNewPoint(QVector<trajectoryNode> node, sensor sensor_model)
{
    QVector3D pos = node[node.size()-1].pos().toQVector3D();
    QQuaternion q = node[node.size()-1].q();

    //LASER
    Qt3DCore::QEntity *laserEntity_ = new Qt3DCore::QEntity(rootEntity);
    Qt3DCore::QTransform *laserTransform_ = new Qt3DCore::QTransform;

    Qt3DExtras::QPhongAlphaMaterial *laserMaterial = new Qt3DExtras::QPhongAlphaMaterial;
    laserMaterial->setDiffuse(QColor(254, 50, 50));
    laserMaterial->setAmbient(QColor(254, 50, 50));
    laserMaterial->setSpecular(QColor(254, 50, 50));
    laserMaterial->setShininess(0.8);
    laserMaterial->setAlpha(0.4);

    float values[9] = {-1,0,0,0,-1,0,0,0,1};
    QMatrix3x3 mat_laserW(values);
    QQuaternion quat_laserW_ = QQuaternion::fromEulerAngles(-90,0,0);
    QMatrix3x3 mat_laserW_ = quat_laserW_.toRotationMatrix();

    QMatrix3x3 mat_aux = q.toRotationMatrix();
    QMatrix3x3 final_rotM = multiplyMatrix3x3(mat_aux,mat_laserW);
    final_rotM = multiplyMatrix3x3(final_rotM,mat_laserW_);
    QQuaternion final_rot = QQuaternion::fromRotationMatrix(final_rotM);
    laserTransform_->setRotation(final_rot);

    QVector3D t(0,-sensor_model.range/2,0);
    QVector3D t_ = pos+final_rot.rotatedVector(t);
    laserTransform_->setTranslation(t_);

    laserEntity_->addComponent(laserMesh);
    laserEntity_->addComponent(laserMaterial);
    laserEntity_->addComponent(laserTransform_);

    lasers.push_back(laserEntity_);

    //SENSOR
    Qt3DCore::QEntity *sensorEntity_ = new Qt3DCore::QEntity(rootEntity);
    Qt3DCore::QTransform *sensorTransform_ = new Qt3DCore::QTransform;

    Qt3DExtras::QPhongAlphaMaterial *modelMaterial = new Qt3DExtras::QPhongAlphaMaterial;
    modelMaterial->setDiffuse(QColor(254,254,254));
    modelMaterial->setAlpha(0.4);

    QVector3D t_s = pos + q.rotatedVector(QVector3D(0,0,-30));
    sensorTransform_->setRotation(q);
    sensorTransform_->setTranslation(t_s);

    sensorEntity_->addComponent(sensorMesh);
    sensorEntity_->addComponent(modelMaterial);
    sensorEntity_->addComponent(sensorTransform_);

    sensors.push_back(sensorEntity_);

    //LINEA
    if (lasers.size()>=2)
    {
        lines.push_back(drawLine(node[node.size()-2].pos().toQVector3D(), node[node.size()-1].pos().toQVector3D(), Qt::black, rootEntity));
    }


}

void myRenderer::drawAxes(QVector3D &origin, Qt3DCore::QEntity *entity)
{

    axis.origin = origin;

    axis.lines_arrows.push_back(drawLine(origin, { origin.x()+300, origin.y(), origin.z() }, Qt::red, entity)); // X
    axis.lines_arrows.push_back(drawLine(origin, { origin.x(), origin.y()+300, origin.z() }, Qt::green, entity)); // Y
    axis.lines_arrows.push_back(drawLine(origin, { origin.x(), origin.y(), origin.z()+300 }, Qt::blue, entity)); // Z

  //X
    axis.arrowX = new Qt3DCore::QEntity(entity);
    axis.arrowX_mesh = new Qt3DExtras::QConeMesh;
    axis.arrowX_transform = new Qt3DCore::QTransform;
    axis.arrowX_Material = new Qt3DExtras::QPhongMaterial;

    axis.arrowX_mesh->setTopRadius(0);
    axis.arrowX_mesh->setBottomRadius(20);
    axis.arrowX_mesh->setLength(40);

    axis.arrowX_Material->setDiffuse(Qt::red);
    axis.arrowX_Material->setAmbient(Qt::red);
    axis.arrowX_Material->setSpecular(Qt::red);
    axis.arrowX_Material->setShininess(0.8);

    axis.arrowX_transform->setTranslation(QVector3D(origin.x()+300,origin.y(),origin.z()));
    axis.arrowX_transform->setRotationZ(-90);

    axis.arrowX->addComponent(axis.arrowX_mesh);
    axis.arrowX->addComponent(axis.arrowX_Material);
    axis.arrowX->addComponent(axis.arrowX_transform);

    //Y
    axis.arrowY = new Qt3DCore::QEntity(entity);
    axis.arrowY_mesh = new Qt3DExtras::QConeMesh;
    axis.arrowY_transform = new Qt3DCore::QTransform;
    axis.arrowY_Material = new Qt3DExtras::QPhongMaterial;

    axis.arrowY_mesh->setTopRadius(0);
    axis.arrowY_mesh->setBottomRadius(20);
    axis.arrowY_mesh->setLength(40);

    axis.arrowY_Material->setDiffuse(Qt::green);
    axis.arrowY_Material->setAmbient(Qt::green);
    axis.arrowY_Material->setSpecular(Qt::green);
    axis.arrowY_Material->setShininess(0.8);

    axis.arrowY_transform->setTranslation(QVector3D(origin.x(),origin.y()+300,origin.z()));

    axis.arrowY->addComponent(axis.arrowY_mesh);
    axis.arrowY->addComponent(axis.arrowY_Material);
    axis.arrowY->addComponent(axis.arrowY_transform);


    //Z
    axis.arrowZ = new Qt3DCore::QEntity(entity);
    axis.arrowZ_mesh = new Qt3DExtras::QConeMesh;
    axis.arrowZ_transform = new Qt3DCore::QTransform;
    axis.arrowZ_Material = new Qt3DExtras::QPhongMaterial;

    axis.arrowZ_mesh->setTopRadius(0);
    axis.arrowZ_mesh->setBottomRadius(20);
    axis.arrowZ_mesh->setLength(40);

    axis.arrowZ_Material->setDiffuse(Qt::blue);
    axis.arrowZ_Material->setAmbient(Qt::blue);
    axis.arrowZ_Material->setSpecular(Qt::blue);
    axis.arrowZ_Material->setShininess(0.8);

    axis.arrowZ_transform->setTranslation(QVector3D(origin.x(),origin.y(),origin.z()+300));
    axis.arrowZ_transform->setRotationX(90);

    axis.arrowZ->addComponent(axis.arrowZ_mesh);
    axis.arrowZ->addComponent(axis.arrowZ_Material);
    axis.arrowZ->addComponent(axis.arrowZ_transform);


}

void myRenderer::updateAxes(QVector3D &origin)
{
    // Clean previous origin and lines
    for (int i=0; i<axis.lines_arrows.size();i++)
        delete axis.lines_arrows[i];

    axis.lines_arrows.clear();

    axis.origin = origin;

    // Draw new Axis
    axis.lines_arrows.push_back(drawLine(origin, { origin.x()+300, origin.y(), origin.z() }, Qt::red, rootEntity)); // X
    axis.lines_arrows.push_back(drawLine(origin, { origin.x(), origin.y()+300, origin.z() }, Qt::green, rootEntity)); // Y
    axis.lines_arrows.push_back(drawLine(origin, { origin.x(), origin.y(), origin.z()+300 }, Qt::blue, rootEntity)); // Z

    axis.arrowX_transform->setTranslation(QVector3D(origin.x()+300,origin.y(),origin.z()));
    axis.arrowY_transform->setTranslation(QVector3D(origin.x(),origin.y()+300,origin.z()));
    axis.arrowZ_transform->setTranslation(QVector3D(origin.x(),origin.y(),origin.z()+300));

    axis.arrowX->addComponent(axis.arrowX_transform);
    axis.arrowY->addComponent(axis.arrowY_transform);
    axis.arrowZ->addComponent(axis.arrowZ_transform);


}

void myRenderer::deletePoints()
{
    for (int i=0; i<lasers.size();i++)
    {
        delete lasers[i];
        delete sensors[i];

        if (i<lasers.size()-1)
            delete lines[i];
    }

    lasers.clear();
    sensors.clear();
    lines.clear();
}

void myRenderer::insertDefect(QVector<QVector3D> pos)
{
    Qt3DExtras::QCuboidMesh *pointMesh = new Qt3DExtras::QCuboidMesh;
    pointMesh->setXExtent(1);
    pointMesh->setYExtent(1);
    pointMesh->setZExtent(1);

    Qt3DExtras::QPhongAlphaMaterial *pointMaterial = new Qt3DExtras::QPhongAlphaMaterial;
    pointMaterial->setDiffuse(QColor(0,0,254));
    pointMaterial->setAlpha(0.8);

    drawDefect(pos,rootEntity);


}

void myRenderer::insertTraj(QVector<QVector3Dd> pos)
{
    QColor color, color_background = view->defaultFrameGraph()->clearColor();

    if (color_background == Qt::white)
        color = Qt::black;
    else
        color = Qt::white;


    trajEntity = drawTraj(pos,rootEntity, color);
}

void myRenderer::deleteTraj()
{
    delete(trajEntity);
}
