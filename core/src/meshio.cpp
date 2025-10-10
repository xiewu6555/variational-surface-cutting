#include <limits>
#include <iostream>
#include <meshio.h>

using namespace std;

bool WavefrontOBJ::write(string filename, Geometry<Euclidean> &geometry)
{
    ofstream out;
    if (!openStream(out, filename))
        return false;

    writeHeader(out, geometry);
    out << "# texture coordinates: NO" << endl;
    cout << endl;

    writeVertices(out, geometry);

    bool useTexCoords = false;
    writeFaces(out, geometry, useTexCoords);

    return true;
}

bool WavefrontOBJ::write(std::string filename, Geometry<Euclidean> &geometry,
                         FaceData<int> &faceMaterials)
{
    ofstream out;
    if (!openStream(out, filename))
        return false;

    writeHeader(out, geometry);
    out << "# texture coordinates: NO" << endl;
    cout << endl;

    writeVertices(out, geometry);

    bool useTexCoords = false;
    writeFaces(out, geometry, faceMaterials);

    return true;
}

bool WavefrontOBJ::write(string filename, Geometry<Euclidean> &geometry, CornerData<Vector2> &texcoords)
{
    ofstream out;
    if (!openStream(out, filename))
        return false;

    writeHeader(out, geometry);
    out << "# texture coordinates: YES" << endl;
    cout << endl;

    writeVertices(out, geometry);
    writeTexCoords(out, geometry, texcoords);

    bool useTexCoords = true;
    writeFaces(out, geometry, useTexCoords);

    return true;
}

bool WavefrontOBJ::write(string filename, Geometry<Euclidean> &geometry, CornerData<Vector2> &texcoords, FaceData<int> &faceMaterials)
{
    ofstream out;
    if (!openStream(out, filename))
        return false;

    writeHeader(out, geometry);
    out << "# texture coordinates: YES" << endl;
    cout << endl;

    writeVertices(out, geometry);
    writeTexCoords(out, geometry, texcoords);

    bool useTexCoords = true;
    writeFaces(out, geometry, faceMaterials, useTexCoords);

    return true;
}

bool WavefrontOBJ::openStream(ofstream &out, string filename)
{
    out.open(filename);

    if (!out.is_open())
    {
        return false;
    }

    // Use full precision---yes, this makes files bigger, but it also
    // means that saving and then re-loading files doesn't result in
    // unexpected behavior.
    out.precision(numeric_limits<double>::max_digits10);

    return true;
}

void WavefrontOBJ::writeHeader(ofstream &out, Geometry<Euclidean> &geometry)
{
    out << "# Mesh exported from GeometryCentral" << endl;
    out << "#  vertices: " << geometry.mesh.nVertices() << endl;
    out << "#     edges: " << geometry.mesh.nEdges() << endl;
    out << "#     faces: " << geometry.mesh.nFaces() << endl;
}

void WavefrontOBJ::writeVertices(ofstream &out, Geometry<Euclidean> &geometry)
{
    HalfedgeMesh &mesh(geometry.mesh);

    for (VertexPtr v : mesh.vertices())
    {
        Vector3 p = geometry.position(v);
        out << "v " << p.x << " " << p.y << " " << p.z << endl;
    }
}

void WavefrontOBJ::writeTexCoords(ofstream &out, Geometry<Euclidean> &geometry,
                                  CornerData<Vector2> &texcoords)
{
    HalfedgeMesh &mesh(geometry.mesh);

    for (CornerPtr c : mesh.corners())
    {
        Vector2 z = texcoords[c];
        out << "vt " << z.x << " " << z.y << endl;
    }
}

void WavefrontOBJ::writeFaces(ofstream &out, Geometry<Euclidean> &geometry, bool useTexCoords)
{
    HalfedgeMesh &mesh(geometry.mesh);

    // Get vertex indices
    VertexData<size_t> indices = mesh.getVertexIndices();

    if (useTexCoords)
    {
        // Get corner indices
        CornerData<size_t> cIndices = mesh.getCornerIndices();

        for (FacePtr f : mesh.faces())
        {
            out << "f";
            for (CornerPtr c : f.adjacentCorners())
            {
                out << " " << indices[c.vertex()] + 1 << "/" << cIndices[c] + 1; // OBJ uses 1-based indexing
            }
            out << endl;
        }
    }
    else
    {
        for (FacePtr f : mesh.faces())
        {
            out << "f";
            for (HalfedgePtr h : f.adjacentHalfedges())
            {
                out << " " << indices[h.vertex()] + 1; // OBJ uses 1-based indexing
            }
            out << endl;
        }
    }
}

void WavefrontOBJ::writeFaces(std::ofstream &out, Geometry<Euclidean> &geometry,
                              FaceData<int> &faceMaterials, bool useTexCoords)
{
    HalfedgeMesh &mesh(geometry.mesh);

    // Get vertex indices
    VertexData<size_t> indices = mesh.getVertexIndices();

    // Get corner indices
    CornerData<size_t> cIndices;
    if (useTexCoords)
    {
        cIndices = mesh.getCornerIndices();
    }

    // Write out the first face group
    int prevFaceMtlIndex = faceMaterials[mesh.face(0)];
    out << "usemtl mtl" << prevFaceMtlIndex << endl;

    {
        for (FacePtr f : mesh.faces())
        {

            // Write out face group, if needed
            if (faceMaterials[f] != prevFaceMtlIndex)
            {
                prevFaceMtlIndex = faceMaterials[f];
                out << "usemtl mtl" << prevFaceMtlIndex << endl;
            }

            if (useTexCoords)
            {
                out << "f";
                for (CornerPtr c : f.adjacentCorners())
                {
                    out << " " << indices[c.vertex()] + 1 << "/" << cIndices[c] + 1; // OBJ uses 1-based indexing
                }
                out << endl;
            }
            else
            {
                out << "f";
                for (HalfedgePtr h : f.adjacentHalfedges())
                {
                    out << " " << indices[h.vertex()] + 1; // OBJ uses 1-based indexing
                }
                out << endl;
            }
        }
    }
}

bool PLY::write(std::string filename, Geometry<Euclidean> &geometry, VertexData<Vector3> colors)
{

    ofstream out;
    out.open(filename);
    if (!out)
    {
        cerr << "Could no open file for writing: " << filename << endl;
        return false;
    }
    out.precision(numeric_limits<double>::max_digits10);

    HalfedgeMesh *mesh = geometry.getMesh();
    size_t nVert = mesh->nVertices();
    size_t nFace = mesh->nFaces();

    // Header
    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << nVert << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "element face " << nFace << endl;
    out << "property list uchar int vertex_index" << endl;
    out << "end_header" << endl;

    auto round255 = [&](double v) { return static_cast<int>(gcClamp(std::round(v * 255), 0.0, 255.)); };

    // Vertices
    for (VertexPtr v : mesh->vertices())
    {
        out << geometry.position(v).x << " ";
        out << geometry.position(v).y << " ";
        out << geometry.position(v).z << " ";
        out << round255(colors[v].x) << " ";
        out << round255(colors[v].y) << " ";
        out << round255(colors[v].z) << endl;
    }

    // Faces
    VertexData<size_t> vInd = mesh->getVertexIndices();
    for (FacePtr f : mesh->faces())
    {
        out << f.degree();
        for (VertexPtr v : f.adjacentVertices())
        {
            out << " " << vInd[v];
        }
        out << endl;
    }

    out.close();

    return true;
}
