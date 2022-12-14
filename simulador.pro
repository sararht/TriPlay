QT       += core gui sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport xml qml

CONFIG += c++20
#CONFIG += sanitizer sanitize_address
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LIBS += -lgomp -lpthread
QMAKE_CXXFLAGS_RELEASE *= -O3
QMAKE_LFLAGS += -Wl,-rpath,"'\$$ORIGIN'"
QMAKE_LFLAGS += -Wl,-rpath,"'/usr/local/lib/'"

INCLUDEPATH += /usr/local/include/mimmo
INCLUDEPATH += /usr/local/include/bitpit
INCLUDEPATH += /usr/local/include/bitpit
INCLUDEPATH += /usr/local/include/vtk-9.0
#INCLUDEPATH += /usr/include/vtk-6.3

#INCLUDEPATH += /usr/local/include/opencv4
INCLUDEPATH += /home/sara/lapack-3.9.0/SRC
#INCLUDEPATH += /usr/local/include/opencascade/
#INCLUDEPATH += /home/sara/VTK/GUISupport/


LIBS += -lmimmo_D -llapacke -llapack -lblas -lbitpit -lgfortran
LIBS += -lmpi -lxml2 -lpetsc -llapacke -llapack -lblas -lgfortran

LIBS += -lvtkGUISupportQt-9.0 -lvtkRenderingOpenGL2-9.0 -lvtkChartsCore-9.0 -lvtkCommonColor-9.0 -lvtkCommonComputationalGeometry-9.0 -lvtkCommonCore-9.0 -lvtkCommonDataModel-9.0 -lvtkCommonExecutionModel-9.0 -lvtkCommonMath-9.0 -lvtkCommonMisc-9.0 -lvtkCommonSystem-9.0 -lvtkCommonTransforms-9.0 -lvtkCommonTransforms-9.0 -lvtkCommonTransforms-9.0 -lvtkDICOMParser-9.0 -lvtkDICOMParser-9.0 -lvtkDICOMParser-9.0 -lvtkDomainsChemistry-9.0 -lvtkDomainsChemistry-9.0 -lvtkDomainsChemistry-9.0 -lvtkdoubleconversion-9.0 -lvtkdoubleconversion-9.0 -lvtkdoubleconversion-9.0 -lvtkexodusII-9.0 -lvtkexodusII-9.0 -lvtkexodusII-9.0 -lvtkexpat-9.0 -lvtkexpat-9.0 -lvtkexpat-9.0 -lvtkFiltersAMR-9.0 -lvtkFiltersAMR-9.0 -lvtkFiltersAMR-9.0 -lvtkFiltersCore-9.0 -lvtkFiltersCore-9.0 -lvtkFiltersCore-9.0 -lvtkFiltersExtraction-9.0 -lvtkFiltersExtraction-9.0 -lvtkFiltersExtraction-9.0 -lvtkFiltersFlowPaths-9.0 -lvtkFiltersFlowPaths-9.0 -lvtkFiltersFlowPaths-9.0 -lvtkFiltersGeneral-9.0 -lvtkFiltersGeneral-9.0 -lvtkFiltersGeneral-9.0 -lvtkFiltersGeneric-9.0 -lvtkFiltersGeneric-9.0 -lvtkFiltersGeneric-9.0 -lvtkFiltersGeometry-9.0 -lvtkFiltersGeometry-9.0 -lvtkFiltersGeometry-9.0 -lvtkFiltersHybrid-9.0 -lvtkFiltersHybrid-9.0 -lvtkFiltersHybrid-9.0 -lvtkFiltersHyperTree-9.0 -lvtkFiltersHyperTree-9.0 -lvtkFiltersHyperTree-9.0 -lvtkFiltersImaging-9.0 -lvtkFiltersImaging-9.0 -lvtkFiltersImaging-9.0 -lvtkFiltersModeling-9.0 -lvtkFiltersModeling-9.0 -lvtkFiltersModeling-9.0 -lvtkFiltersParallel-9.0 -lvtkFiltersParallel-9.0 -lvtkFiltersParallel-9.0 -lvtkFiltersParallelImaging-9.0 -lvtkFiltersParallelImaging-9.0 -lvtkFiltersParallelImaging-9.0 -lvtkFiltersPoints-9.0 -lvtkFiltersPoints-9.0 -lvtkFiltersPoints-9.0 -lvtkFiltersProgrammable-9.0 -lvtkFiltersProgrammable-9.0 -lvtkFiltersProgrammable-9.0 -lvtkFiltersSelection-9.0 -lvtkFiltersSelection-9.0 -lvtkFiltersSelection-9.0 -lvtkFiltersSMP-9.0 -lvtkFiltersSMP-9.0 -lvtkFiltersSMP-9.0 -lvtkFiltersSources-9.0 -lvtkFiltersSources-9.0 -lvtkFiltersSources-9.0 -lvtkFiltersStatistics-9.0 -lvtkFiltersStatistics-9.0 -lvtkFiltersStatistics-9.0 -lvtkFiltersTexture-9.0 -lvtkFiltersTexture-9.0 -lvtkFiltersTexture-9.0 -lvtkFiltersTopology-9.0 -lvtkFiltersTopology-9.0 -lvtkFiltersTopology-9.0 -lvtkFiltersVerdict-9.0 -lvtkFiltersVerdict-9.0 -lvtkFiltersVerdict-9.0 -lvtkfreetype-9.0 -lvtkfreetype-9.0 -lvtkfreetype-9.0 -lvtkGeovisCore-9.0 -lvtkGeovisCore-9.0 -lvtkGeovisCore-9.0 -lvtkgl2ps-9.0 -lvtkgl2ps-9.0 -lvtkgl2ps-9.0 -lvtkglew-9.0 -lvtkglew-9.0 -lvtkglew-9.0 -lvtkhdf5-9.0 -lvtkhdf5-9.0 -lvtkhdf5-9.0 -lvtkhdf5_hl-9.0 -lvtkhdf5_hl-9.0 -lvtkhdf5_hl-9.0 -lvtkImagingColor-9.0 -lvtkImagingColor-9.0 -lvtkImagingColor-9.0 -lvtkImagingCore-9.0 -lvtkImagingCore-9.0 -lvtkImagingCore-9.0 -lvtkImagingFourier-9.0 -lvtkImagingFourier-9.0 -lvtkImagingFourier-9.0 -lvtkImagingGeneral-9.0 -lvtkImagingGeneral-9.0 -lvtkImagingGeneral-9.0 -lvtkImagingHybrid-9.0 -lvtkImagingHybrid-9.0 -lvtkImagingHybrid-9.0 -lvtkImagingMath-9.0 -lvtkImagingMath-9.0 -lvtkImagingMath-9.0 -lvtkImagingMorphological-9.0 -lvtkImagingMorphological-9.0 -lvtkImagingMorphological-9.0 -lvtkImagingSources-9.0 -lvtkImagingSources-9.0 -lvtkImagingSources-9.0 -lvtkImagingStatistics-9.0 -lvtkImagingStatistics-9.0 -lvtkImagingStatistics-9.0 -lvtkImagingStencil-9.0 -lvtkImagingStencil-9.0 -lvtkImagingStencil-9.0 -lvtkInfovisCore-9.0 -lvtkInfovisCore-9.0 -lvtkInfovisCore-9.0 -lvtkInfovisLayout-9.0 -lvtkInfovisLayout-9.0 -lvtkInfovisLayout-9.0 -lvtkInteractionImage-9.0 -lvtkInteractionImage-9.0 -lvtkInteractionImage-9.0 -lvtkInteractionStyle-9.0 -lvtkInteractionStyle-9.0 -lvtkInteractionStyle-9.0 -lvtkInteractionWidgets-9.0 -lvtkInteractionWidgets-9.0 -lvtkInteractionWidgets-9.0 -lvtkIOAMR-9.0 -lvtkIOAMR-9.0 -lvtkIOAMR-9.0 -lvtkIOAsynchronous-9.0 -lvtkIOAsynchronous-9.0 -lvtkIOAsynchronous-9.0 -lvtkIOCityGML-9.0 -lvtkIOCityGML-9.0 -lvtkIOCityGML-9.0 -lvtkIOCore-9.0 -lvtkIOCore-9.0 -lvtkIOCore-9.0 -lvtkIOEnSight-9.0 -lvtkIOEnSight-9.0 -lvtkIOEnSight-9.0 -lvtkIOExodus-9.0 -lvtkIOExodus-9.0 -lvtkIOExodus-9.0 -lvtkIOExport-9.0 -lvtkIOExport-9.0 -lvtkIOExport-9.0 -lvtkIOExportGL2PS-9.0 -lvtkIOExportGL2PS-9.0 -lvtkIOExportGL2PS-9.0 -lvtkIOExportPDF-9.0 -lvtkIOExportPDF-9.0 -lvtkIOExportPDF-9.0 -lvtkIOGeometry-9.0 -lvtkIOGeometry-9.0 -lvtkIOGeometry-9.0 -lvtkIOImage-9.0 -lvtkIOImage-9.0 -lvtkIOImage-9.0 -lvtkIOImport-9.0 -lvtkIOImport-9.0 -lvtkIOImport-9.0 -lvtkIOInfovis-9.0 -lvtkIOInfovis-9.0 -lvtkIOInfovis-9.0 -lvtkIOLegacy-9.0 -lvtkIOLegacy-9.0 -lvtkIOLegacy-9.0 -lvtkIOLSDyna-9.0 -lvtkIOLSDyna-9.0 -lvtkIOLSDyna-9.0 -lvtkIOMINC-9.0 -lvtkIOMINC-9.0 -lvtkIOMINC-9.0 -lvtkIOMotionFX-9.0 -lvtkIOMotionFX-9.0 -lvtkIOMotionFX-9.0 -lvtkIOMovie-9.0 -lvtkIOMovie-9.0 -lvtkIOMovie-9.0 -lvtkIONetCDF-9.0 -lvtkIONetCDF-9.0 -lvtkIONetCDF-9.0 -lvtkIOOggTheora-9.0 -lvtkIOOggTheora-9.0 -lvtkIOOggTheora-9.0 -lvtkIOParallel-9.0 -lvtkIOParallel-9.0 -lvtkIOParallel-9.0 -lvtkIOParallelXML-9.0 -lvtkIOParallelXML-9.0 -lvtkIOParallelXML-9.0 -lvtkIOPLY-9.0 -lvtkIOPLY-9.0 -lvtkIOPLY-9.0 -lvtkIOSegY-9.0 -lvtkIOSegY-9.0 -lvtkIOSegY-9.0 -lvtkIOSQL-9.0 -lvtkIOSQL-9.0 -lvtkIOSQL-9.0 -lvtkIOTecplotTable-9.0 -lvtkIOTecplotTable-9.0 -lvtkIOTecplotTable-9.0 -lvtkIOVeraOut-9.0 -lvtkIOVeraOut-9.0 -lvtkIOVeraOut-9.0 -lvtkIOVideo-9.0 -lvtkIOVideo-9.0 -lvtkIOVideo-9.0 -lvtkIOXML-9.0 -lvtkIOXML-9.0 -lvtkIOXML-9.0 -lvtkIOXMLParser-9.0 -lvtkIOXMLParser-9.0 -lvtkIOXMLParser-9.0 -lvtkjpeg-9.0 -lvtkjpeg-9.0 -lvtkjpeg-9.0 -lvtkjsoncpp-9.0 -lvtkjsoncpp-9.0 -lvtkjsoncpp-9.0 -lvtklibharu-9.0 -lvtklibharu-9.0 -lvtklibharu-9.0 -lvtklibproj-9.0 -lvtklibproj-9.0 -lvtklibproj-9.0 -lvtklibxml2-9.0 -lvtklibxml2-9.0 -lvtklibxml2-9.0 -lvtkloguru-9.0 -lvtkloguru-9.0 -lvtkloguru-9.0 -lvtklz4-9.0 -lvtklz4-9.0 -lvtklz4-9.0 -lvtklzma-9.0 -lvtklzma-9.0 -lvtklzma-9.0 -lvtkmetaio-9.0 -lvtkmetaio-9.0 -lvtkmetaio-9.0 -lvtknetcdf-9.0 -lvtknetcdf-9.0 -lvtknetcdf-9.0 -lvtkogg-9.0 -lvtkogg-9.0 -lvtkogg-9.0 -lvtkParallelCore-9.0 -lvtkParallelCore-9.0 -lvtkParallelCore-9.0 -lvtkParallelDIY-9.0 -lvtkParallelDIY-9.0 -lvtkParallelDIY-9.0 -lvtkpng-9.0 -lvtkpng-9.0 -lvtkpng-9.0 -lvtkpugixml-9.0 -lvtkpugixml-9.0 -lvtkpugixml-9.0 -lvtkRenderingAnnotation-9.0 -lvtkRenderingAnnotation-9.0 -lvtkRenderingAnnotation-9.0 -lvtkRenderingContext2D-9.0 -lvtkRenderingContext2D-9.0 -lvtkRenderingContext2D-9.0 -lvtkRenderingCore-9.0 -lvtkRenderingCore-9.0 -lvtkRenderingCore-9.0 -lvtkRenderingFreeType-9.0 -lvtkRenderingFreeType-9.0 -lvtkRenderingFreeType-9.0 -lvtkRenderingImage-9.0 -lvtkRenderingImage-9.0 -lvtkRenderingImage-9.0 -lvtkRenderingLabel-9.0 -lvtkRenderingLabel-9.0 -lvtkRenderingLabel-9.0 -lvtkRenderingLOD-9.0 -lvtkRenderingLOD-9.0 -lvtkRenderingLOD-9.0 -lvtkRenderingSceneGraph-9.0 -lvtkRenderingSceneGraph-9.0 -lvtkRenderingSceneGraph-9.0 -lvtkRenderingUI-9.0 -lvtkRenderingUI-9.0 -lvtkRenderingUI-9.0 -lvtkRenderingVolume-9.0 -lvtkRenderingVolume-9.0 -lvtkRenderingVolume-9.0 -lvtkRenderingVolumeOpenGL2-9.0 -lvtkRenderingVolumeOpenGL2-9.0 -lvtkRenderingVolumeOpenGL2-9.0 -lvtkRenderingVtkJS-9.0 -lvtkRenderingVtkJS-9.0 -lvtkRenderingVtkJS-9.0 -lvtksqlite-9.0 -lvtksqlite-9.0 -lvtksqlite-9.0 -lvtksys-9.0 -lvtksys-9.0 -lvtksys-9.0 -lvtkTestingRendering-9.0 -lvtkTestingRendering-9.0 -lvtkTestingRendering-9.0 -lvtktheora-9.0 -lvtktheora-9.0 -lvtktheora-9.0 -lvtktiff-9.0 -lvtktiff-9.0 -lvtktiff-9.0 -lvtkverdict-9.0 -lvtkverdict-9.0 -lvtkverdict-9.0 -lvtkViewsContext2D-9.0 -lvtkViewsContext2D-9.0 -lvtkViewsContext2D-9.0 -lvtkViewsCore-9.0 -lvtkViewsCore-9.0 -lvtkViewsCore-9.0 -lvtkViewsInfovis-9.0 -lvtkViewsInfovis-9.0 -lvtkViewsInfovis-9.0 -lvtkWrappingTools-9.0 -lvtkWrappingTools-9.0 -lvtkWrappingTools-9.0 -lvtkzlib-9.0 -lvtkzlib-9.0 -lvtkzlib-9.0
#-lvtkRenderingOpenGL2-9.0 -lvtkRenderingOpenGL2-9.0 -lvtkRenderingOpenGL2-9.0

LIBS += -L/usr/local/lib/ -lTKernel -lTKMath -lTKService -lTKV3d -lTKOpenGl \
        -lTKBRep -lTKSTEP -lTKSTEPAttr -lTKSTEP209 -lTKTopAlgo \
        -lTKSTEPBase -lTKXSBase -lTKPrim -lTKBool -lTKV3d -lTKCAF -lTKLCAF -lTKXDESTEP -lTKXCAF -lTKGeomAlgo -lTKG3d -lTKGeomBase


LIBS += $(shell pkg-config opencv --libs)

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    KD_tree_cpp/boundingbox.cpp \
    KD_tree_cpp/kdnode.cpp \
    KD_tree_cpp/triangle.cpp \
    MainWindow.cpp \
    basicplot.cpp \
    controller.cpp \
    fitplane3d.cpp \
    include_defects/bumpdefect.cpp \
    include_defects/crackdefect.cpp \
    include_defects/defect.cpp \
    include_defects/lossdefect.cpp \
    include_defects/pickdefect.cpp \
    main.cpp \
    prueba.cpp \
    qcustomplot.cpp \
    qvector3dd.cpp \
    render.cpp \
    sensor.cpp \
    trajectorycontroller.cpp \
    trajectorynode.cpp \
    include_defects/personalizeddefect.cpp \
    vtktools.cpp \
    rendervtk.cpp \
    defectselection.cpp \
    cv_perlin_noise/PerlinNoise.cpp

HEADERS += \
    KD_tree_cpp/boundingbox.h \
    KD_tree_cpp/kdnode.h \
    KD_tree_cpp/triangle.h \
    MainWindow.h \
    basicplot.h \
    bestFitPlane/vtkBestFitPlane.h \
    controller.h \
    fitplane3d.h \
    include_defects/bumpdefect.h \
    include_defects/crackdefect.h \
    include_defects/defect.h \
    include_defects/lossdefect.h \
    include_defects/pickdefect.h \
    kdtree_c.h \
    prueba.h \
    qcustomplot.h \
    qvector3dd.h \
    render.h \
    sensor.h \
    stl_reader.h \
    trajectorycontroller.h \
    trajectorynode.h \
    include_defects/personalizeddefect.h \
    vtktools.h \
    rendervtk.h \
    mouseinteractorstyle.h \
    defectselection.h \
    draginteractorstyle.h \
    cv_perlin_noise/PerlinNoise.h

FORMS += \
    MainWindow.ui \
    basicplot.ui \
    sensorParameters.ui \
    defectselection.ui


LIBS+= -lOpenCL -lpython2.7
QT += 3dcore 3drender 3dinput 3dlogic 3dextras 3danimation
# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    kernels/kernel.cl \
    kernels/kernel_filter.cl \
    kernels/kernel_kdtree.cl \
    kernels/kernel_points.cl

RESOURCES += \
    MyRes.qrc