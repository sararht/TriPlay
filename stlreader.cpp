#include "stlreader.h"

STLReader::STLReader()
{

}


vertex operator ^ (const vertex & a, const vertex & b) {
    // Cross product.
    vertex result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return(result);
}

vertex operator - (const vertex & a, const vertex & b) {
    // Subtraction.
    vertex result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return(result);
}

void STLReader::STLAddFacet(vertex v1, vertex v2, vertex v3, vertex normal) {


    // STL facet implementation goes here.

    cout << v1.x << " " << v1.y << " " << v1.z << endl;

}

void STLReader::STLImport(const string & fileName) {

    // Import a binary STL file.

    int nVertex = 0; // Number of vertices read.
    int nFacet = 0;  // Number of facets read.

    // Open the file for reading using an input fstream.

    ifstream ifs(fileName, ifstream::binary);

    // Get pointer to the associated buffer object.
    // rdbuf returns a streambuf object associated with the
    // input fstream object ifs.

    filebuf* pbuf = ifs.rdbuf();

    // Calculate the file's size.

    auto size = pbuf->pubseekoff(0, ifs.end);

    // Set the position pointer to the beginning of the file.

    pbuf->pubseekpos(0);

    // Allocate memory to contain file data.

    char* buffer = new char[(size_t)size];

    // Get file data. sgetn grabs all the characters from the streambuf
    // object 'pbuf'. The return value of sgetn is the number of characters
    // obtained - ordinarily, this value should be checked for equality
    // against the number of characters requested.

    pbuf->sgetn(buffer, size);

    // Test to see if the file is binary.

    //if (!isBinarySTL(buffer)) return;

    char * bufptr = buffer;

    bufptr += 80;  // Skip past the header.
    bufptr += 4;   // Skip past the number of triangles.

    vertex normal;
    vertex v1, v2, v3;

    while (bufptr < buffer + size) {

        normal.x = *(float *)(bufptr);
        normal.y = *(float *)(bufptr + 4);
        normal.z = *(float *)(bufptr + 8);
        bufptr += 12;

        v1.x = *(float *)(bufptr);
        v1.y = *(float *)(bufptr + 4);
        v1.z = *(float *)(bufptr + 8);
        bufptr += 12;

        v2.x = *(float *)(bufptr);
        v2.y = *(float *)(bufptr + 4);
        v2.z = *(float *)(bufptr + 8);
        bufptr += 12;

        v3.x = *(float *)(bufptr);
        v3.y = *(float *)(bufptr + 4);
        v3.z = *(float *)(bufptr + 8);
        bufptr += 12;

        const float eps = (float) 1.0e-9;

        // If the normal in the STL file is blank, then create a proper normal.

        if (abs(normal.x) < eps && abs(normal.y) < eps && abs(normal.z) < eps) {
            vertex u, v;
            u = v2 - v1;
            v = v3 - v1;
            normal = u ^ v;
            unit(normal);
        }

        nFacet++;
        nVertex += 3;

      //  void STLAddFacet(vertex, vertex, vertex, vertex);
      //  STLAddFacet(v1, v2, v3, normal);

        bufptr += 2;
    }


    ifs.close();

    delete[] buffer;
}





void STLReader::unit(vertex & v) {
    // Normalize a vector.
    float vmod = pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2);
    vmod = sqrt(vmod);

    if (vmod > (float)1.0e-9) {
        v.x /= vmod;
        v.y /= vmod;
        v.z /= vmod;
    }
}

