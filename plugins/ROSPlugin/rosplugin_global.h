#ifndef ROSPLUGIN_GLOBAL_H
#define ROSPLUGIN_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ROSPLUGIN_LIBRARY)
#  define ROSPLUGINSHARED_EXPORT Q_DECL_EXPORT
#else
#  define ROSPLUGINSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // ROSPLUGIN_GLOBAL_H
