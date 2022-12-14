#include "crackdefect.h"

bool SortForMinDirection(const QVector3D &a, const QVector3D &b, int xyz)
{
    if (xyz == 0) return (a.x() < b.x());
    else if (xyz == 1) return (a.y() < b.y());
    else if (xyz == 2) return (a.z() < b.z());
}

class PointsSorter
{
    int direction_;
public:
    PointsSorter(int direction){direction_=direction;}
    bool operator()(QVector3D p1, QVector3D p2) const{
        return SortForMinDirection(p1,p2,direction_);}
};




CrackDefect::CrackDefect(){}
CrackDefect::CrackDefect(QVector<QVector3D> points_, float depth_, float width_crack_, QVector3D normal_, int n_columns_first_, int n_columns_last_)
{
    points = points_;
    n_points = points.size();
    n_columns_first = n_columns_first_;
    n_columns_last = n_columns_last_;
    width_crack = width_crack_;

    QVector3D posxyz_;
    float length_ = 0.0, width_ = 0.0;


    if (fabs(normal_.x()) == 1)
    {
        float miny=points[0].y(), maxy=points[0].y(), minz=points[0].z(), maxz=points[0].z();
        for (int i=1; i<n_points; i++)
        {
            if(points[i].y()<miny)
               miny = points[i].y();
            if(points[i].z()<minz)
               minz = points[i].z();

            if(points[i].y()>maxy)
               maxy = points[i].y();
            if(points[i].z()>maxz)
               maxz = points[i].z();
        }

        float longy=maxy-miny;
        float longz=maxz-minz;

        length_ = std::max(longy,longz);
        width_ = std::min(longy,longz);

        posxyz_.setX(points[0].x());
        posxyz_.setY(miny+longy/2);
        posxyz_.setZ(minz+longz/2);

        if (longy>=longz)
            direction = 1;
        else
            direction = 2;

    }

    else if (fabs(normal_.y()) == 1)
    {
        float minx=points[0].x(), maxx=points[0].x(), minz=points[0].z(), maxz=points[0].z();
        for (int i=1; i<n_points; i++)
        {
            if(points[i].x()<minx)
               minx = points[i].x();
            if(points[i].z()<minz)
               minz = points[i].z();

            if(points[i].x()>maxx)
               maxx = points[i].x();
            if(points[i].z()>maxz)
               maxz = points[i].z();
        }

        float longx=maxx-minx;
        float longz=maxz-minz;

        length_ = std::max(longx,longz);
        width_ = std::min(longx,longz);

        posxyz_.setX(minx+longx/2);
        posxyz_.setY(points[0].y());
        posxyz_.setZ(minz+longz/2);

        if (longx>=longz)
            direction = 0;
        else
            direction = 2;

    }

    else if (fabs(normal_.z()) == 1)
    {
        float minx=points[0].x(), maxx=points[0].x(), miny=points[0].y(), maxy=points[0].y();
        for (int i=1; i<n_points; i++)
        {
            if(points[i].x()<minx)
               minx = points[i].x();
            if(points[i].y()<miny)
               miny = points[i].y();

            if(points[i].x()>maxx)
               maxx = points[i].x();
            if(points[i].y()>maxy)
               maxy = points[i].y();
        }

        float longx=maxx-minx;
        float longy=maxy-miny;

        length_ = std::max(longx,longy);
        width_ = std::min(longx,longy);

        posxyz_.setX(minx+longx/2);
        posxyz_.setY(miny+longy/2);
        posxyz_.setZ(points[0].z());

        if (longx>=longy)
            direction = 0;
        else
            direction = 1;


    }

    std::sort(points.begin(), points.end(), PointsSorter(direction));
    posxyz = posxyz_;
    length = length_;
    width = width_ +width_crack+20;
    depth = depth_;
    normal = normal_;
    dir_deg = 0;

}

CrackDefect::CrackDefect(QVector3D origin, QVector3D end, QVector<float> distributionL,QVector<float> distributionW,QVector3D normal_, float depth_, float width_crack_, int n_columns_first_, int n_columns_last_)
{
    //Calcular los puntos
    QVector<QVector3D> points_;

    CrackDefect(points_,depth_,width_crack_,normal_,n_columns_first_,n_columns_last_);
}

bool isPointInRect(QVector2D p, float m, float n)
{
    if((p.y() >= m*p.x()+n-2 && p.y() < m*p.x()+n+2) || (p.y() <= m*p.x()+n-2 && p.y() > m*p.x()+n+2))
        return true;
    else
        return false;
}

struct grieta_xy
{
    QVector2D pos_lattice;
    QVector2D pos_cartesian_local;
    int line;
};

void CrackDefect::fillLattice()
{
    std::cout << "Fill Lattice" << std::endl;
    // Get span and dimensions of the lattice
    darray3E span = lattice->getSpan();
    iarray3E dim = lattice->getDimension();

    std::cout << "DIM: " <<dim[0] << ", "<<dim[1]<<", "<<dim[2]<<std::endl;

    // Variables
    double cte_x, cte_y;
    QVector<float> m,n;
    QVector2D pointFirst, pointLast, pointFirst_rel, pointLast_rel, posxy;
    QVector<QVector2D> points_rel;

   // dvecarr3E displ;

    // Obtain variables depending on the normal direction
    if (fabs(normal.x()) == 1)
    {
        posxy = QVector2D(posxyz.y(), posxyz.z());

        pointFirst = QVector2D(points[0].y(), points[0].z());
        pointLast = QVector2D(points[n_points-1].y(), points[n_points-1].z());

        pointFirst_rel = pointFirst - posxy;
        pointLast_rel = pointLast - posxy;

        for (int i=0; i<n_points; i++)
        {
            QVector2D point_ = QVector2D(points[i].y(), points[i].z());
            QVector2D point_rel_ = point_ - posxy;

            points_rel.push_back(point_rel_);
        }
    }
    else if (fabs(normal.y())==1)
    {
        posxy = QVector2D(posxyz.x(), posxyz.z());

        pointFirst = QVector2D(points[0].x(), points[0].z());
        pointLast = QVector2D(points[n_points-1].x(), points[n_points-1].z());

        pointFirst_rel = pointFirst - posxy;
        pointLast_rel = pointLast - posxy;

        for (int i=0; i<n_points; i++)
        {
            QVector2D point_ = QVector2D(points[i].x(), points[i].z());
            QVector2D point_rel_ = point_ - posxy;

            points_rel.push_back(point_rel_);
        }
    }

    else if(fabs(normal.z())==1)
    {
        posxy = QVector2D(posxyz.x(), posxyz.y());

        pointFirst = QVector2D(points[0].x(), points[0].y());
        pointLast = QVector2D(points[n_points-1].x(), points[n_points-1].y());

        pointFirst_rel = pointFirst - posxy;
        pointLast_rel = pointLast - posxy;

        for (int i=0; i<n_points; i++)
        {
            QVector2D point_ = QVector2D(points[i].x(), points[i].y());
            QVector2D point_rel_ = point_ - posxy;

            points_rel.push_back(point_rel_);
        }
    }




        if ( (abs(normal.x())==1 && direction == 1) || (abs(normal.y())==1 && direction == 0) || (abs(normal.z()) && direction == 0))
        {
            std::cout << "Fill Lattice0" << std::endl;

            dim[1] = span[1]*2;
            //dim[1] = 2;
            dim[2] = span[2]*3;//500;//span[2];
            if (dim[2]<300)
                dim[2]=300;

            lattice->setDimension(dim); // Update dimensions
            lattice->build();

            std::cout << "Fill Lattice0.1" << std::endl;


            cte_x = span[1]/dim[1];
            cte_y = span[2]/dim[2];

            for (int i=1; i<n_points;i++)
            {
                QVector2D point1_rel = points_rel[i-1];
                QVector2D point2_rel = points_rel[i];

                float m_ = (point1_rel.y()-point2_rel.y())/(point1_rel.x()-point2_rel.x());
                float n_ = point1_rel.y() - m_*point1_rel.x();

                m.push_back(m_);
                n.push_back(n_);
            }

            // Fill lattice
            int ndof = lattice->getNNodes();
            displ = dvecarr3E(ndof, darray3E{0,0,0});

            std::cout << "Fill Lattice0.2" << std::endl;

            QVector<grieta_xy> pointsGrieta;

            bool inRect=false;
            float x,y;
            for(int l0=0; l0<dim[1]; l0++) //columnas
            {
                //Calcular max desviación de posición aleatoria del centro Y respecto a la línea que une 2 pts
                //Calcular máx desviación de variacion aleatoria de profundidad
                //Calcular máx desviación aleatoria de ancho

                //float depth;
                for (int l2=0; l2<dim[2]; l2++) //filas
                {
                    for (int l1=0; l1<dim[0]; l1++) //profundidad
                    {
                        x = - span[1]/2 + cte_x*l0;
                        y = - span[2]/2 + cte_y*l2;
                        //int index = lattice->accessPointIndex(l0,l1,l2);
                        for(int i_line=0;i_line<m.size();i_line++)
                        {
                            if (isPointInRect(QVector2D(x,y),m[i_line],n[i_line]) && (x >= points_rel[i_line].x() && x <= points_rel[i_line+1].x()))
                            {
                                pointsGrieta.push_back(grieta_xy{QVector2D(l0,l2), QVector2D{x,y}, i_line});
                                inRect = true;
                                continue;
                            }

                        }//for
                    }
                    if (inRect)
                    {
                        inRect=false;
                        continue;
                    }
                }
            }

            std::cout << "Fill Lattice0.3" << std::endl;

            float sigmax = dim[1]/7;
            float sigmay = width_crack/7; //CALCULAR PARA CADA COLUMNA SEGÚN WIDTH_CRACK
            int n_cols = dim[1];
            int n_rows = dim[2];

            for (int i=0; i< pointsGrieta.size(); i++)
            {
                int col = pointsGrieta[i].pos_lattice.x();
                float xm_ = pointsGrieta[i].pos_cartesian_local.x();
                float ym_ = pointsGrieta[i].pos_cartesian_local.y();

                //float maxYm_d, maxDepth_d, maxWidth_d;

                //Cómo defimo los máximos? Cómo me venga en gana? No sé!!! :(
                //Una vez definidos utilzarlos para calcular las desviaciones aleatorias

                float depth_i;
                if ( col > n_columns_first && col < (n_cols - n_columns_last))
                    depth_i = depth;
                else if ( col > n_columns_first) //Last columns
                {
                    depth_i = depth-(n_columns_last-(n_cols-col))*(depth/n_columns_last);
                    sigmay=width_crack/17;
                    sigmax = dim[1]/17;

                }
                else //First columns
                {
                    depth_i = col*depth/n_columns_first;//depth-(n_columns_first-col)*(depth/n_columns_first);
                    sigmay=width_crack/17;
                    sigmax = dim[1]/17;

                }



                for (int row=0; row<n_rows; row++) //No sean todo, sino el width crack¿? -> Creo que vale así, definí sigma para tener el valor del width_crack
                {
                    //int r =rand() % 3 -1;
                    //ym_ = ym_+(r*cte_y/100);
                    double x=pointsGrieta[i].pos_cartesian_local.x();;
                    double y= - span[2]/2 + cte_y*row;
                    //La gaussiana se centra en posGrieta.pos
                    double z=depth_i*exp(-((x-xm_)*(x-xm_)/(2*sigmax*sigmax)) - ((y-ym_)*(y-ym_)/(2*sigmay*sigmay)));

                 //   if (abs(z)>abs(3*depth/4)) z=std::numeric_limits<double>::quiet_NaN();

                    for (int d=0; d<dim[0]; d++)
                    {
                        int index = lattice->accessPointIndex(d, col, row);
                        displ[index][0]= -z;
                    }

                }
            }
    }//if(direction==0)

        if ((normal.x()==1 && direction == 2) || (normal.y()==1 && direction == 2) || (normal.z() && direction == 1))
        {
            std::cout << "Fill Lattice1" << std::endl;

            float aux = span[2];
            span[2] = span[1];
            span[1]= aux;

            dim[1] = span[1]*3;
            //dim[1] = 2;
            dim[2] = span[2];//500;//span[2];
            if (dim[1]<300)
                dim[1]=300;

            lattice->setDimension(dim); // Update dimensions
            lattice->setSpan(span);
            lattice->build();
            std::cout << "Fill Lattice1.1" << std::endl;


            cte_x = span[1]/dim[1];
            cte_y = span[2]/dim[2];


            for (int i=1; i<n_points;i++)
            {
                QVector2D point1_rel = points_rel[i-1];
                QVector2D point2_rel = points_rel[i];

                float m_ = (point1_rel.y()-point2_rel.y())/(point1_rel.x()-point2_rel.x());
                float n_ = point1_rel.y() - m_*point1_rel.x();

                m.push_back(m_);
                n.push_back(n_);
            }

            std::cout << "Fill Lattice1.2" << std::endl;

            // Fill lattice
            int ndof = lattice->getNNodes();
            displ = dvecarr3E(ndof, darray3E{0,0,0});

            QVector<grieta_xy> pointsGrieta;

            bool inRect=false;
            float x,y;
            for(int l2=0; l2<dim[2]; l2++) //filas
            {
                //Calcular max desviación de posición aleatoria del centro Y respecto a la línea que une 2 pts
                //Calcular máx desviación de variacion aleatoria de profundidad
                //Calcular máx desviación aleatoria de ancho

                //float depth;
                for (int l0=0; l0<dim[1]; l0++) //columnas
                {
                    for (int l1=0; l1<dim[0]; l1++) //profundidad
                    {
                        x = - span[1]/2 + cte_x*l0;
                        y = - span[2]/2 + cte_y*l2;
                       // int index = lattice->accessPointIndex(l0,l1,l2);
                        for(int i_line=0;i_line<m.size();i_line++)
                        {
                            if (isPointInRect(QVector2D(x,y),m[i_line],n[i_line]) && (y >= points_rel[i_line].y() && y <= points_rel[i_line+1].y()))
                            {
                                pointsGrieta.push_back(grieta_xy{QVector2D(l0,l2), QVector2D{x,y}, i_line});
                                inRect = true;
                                continue;
                            }

                        }//for
                    }
                    if (inRect)
                    {
                        inRect=false;
                        continue;
                    }
                }
            }

            std::cout << "Fill Lattice1.3" << std::endl;

            float sigmax = width_crack/6;
            float sigmay = 1; //CALCULAR PARA CADA COLUMNA SEGÚN WIDTH_CRACK
            int n_cols = dim[1];
            int n_rows = dim[2];

            for (int i=0; i< pointsGrieta.size(); i++)
            {
                int row = pointsGrieta[i].pos_lattice.y();
                float xm_ = pointsGrieta[i].pos_cartesian_local.x();
                float ym_ = pointsGrieta[i].pos_cartesian_local.y();

                //float maxYm_d, maxDepth_d, maxWidth_d;

                //Cómo defino los máximos?
                //Una vez definidos utilzarlos para calcular las desviaciones aleatorias

                float depth_i;
                if ( row > n_columns_first && row < (n_rows - n_columns_last))
                    depth_i = depth;
                else if ( row > n_columns_first) //Last columns
                {
                    depth_i = depth-(n_columns_last-(n_rows-row))*(depth/n_columns_last);
                }
                else //First columns
                    depth_i = depth-(n_columns_first-row)*(depth/n_columns_first);


                for (int col=0; col<n_cols; col++) //No sean todo, sino el width crack¿? -> Creo que vale así, definí sigma para tener el valor del width_crack
                {
                    //int r =rand() % 3 -1;
                  //  xm_ = xm_+r*cte_x;
                    double x=- span[1]/2 + cte_x*col;
                    double y= pointsGrieta[i].pos_cartesian_local.y();
                    //La gaussiana se centra en posGrieta.pos
                    double z=depth_i*exp(-((x-xm_)*(x-xm_)/(2*sigmax*sigmax)) - ((y-ym_)*(y-ym_)/(2*sigmay*sigmay)));

                    //if (abs(z)>1)
                        //int a=0;

                    for (int d=0; d<dim[0]; d++)
                    {
                        int index = lattice->accessPointIndex(d, col, row);
                        displ[index][0]= -z;

                    }

                }
            }
    }//if(direction==2)





//------------------



        std::cout << "Fill Lattice2" << std::endl;

    // Set Generic input block with the displacements defined above.
    input = new mimmo::GenericInput();
    input->setReadFromFile(false);
    input->setInput(displ);

    // Set Generic output block to write the displacements defined above.
    output = new mimmo::GenericOutput();
    output->setFilename("manipulators_output_00003.csv");
    output->setCSV(true);
    std::cout << "Fill Lattice3" << std::endl;

}

void CrackDefect::createDefectVTK()
{}
