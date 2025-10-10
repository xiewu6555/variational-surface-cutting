#include "local_feature_size.h"

#include <string>
#include <stdexcept>
#include "fast_marching_method.h"

VertexData<double> computeLocalFeatureSize_eikonal(Geometry<Euclidean>* geometry) {
    HalfedgeMesh* mesh = geometry->getMesh();
    
    // Minimum feature size throughout the shape
    // TODO not the most principled value; orientation dependent
    double minCurvature = 1.0 / std::pow(geometry->lengthScale(), 2.0);
    VertexData<double> featureCurvature(mesh);

    for(VertexPtr v : mesh->vertices()) {
        
        // Compute pointwise curvature
        double ptwiseK = geometry->angleDefect(v) / geometry->area(v.dual());
        if(v.isBoundary()) {
            ptwiseK = 0.0; // TODO decide what to do, if anything, about boundary curvature
        }

        double ptwiseCurvatureMag = std::abs(ptwiseK);
        featureCurvature[v] = std::max(minCurvature, ptwiseCurvatureMag);
    }


    // Build the FMM input
    std::vector<std::pair<VertexPtr, double>> initDistances;
    for(VertexPtr v : mesh->vertices()) {
        double rawFeatureSize = 1.0 / std::sqrt(featureCurvature[v]);
        initDistances.push_back(std::make_pair(v, rawFeatureSize));
    }

    // Do an FMM like expansion on the principle that feature size decays at a rate of 1 distance per distance
    VertexData<double> featureSize = GC::FMMDistance(geometry, initDistances);
    // VertexData<double> featureSize = featureCurvature;

    return featureSize;
}

VertexData<double> computeLocalFeatureSize_smooth(Geometry<Euclidean>* geometry, GC::SparseMatrix<double>& zeroFormLaplacian, double smoothing) {
    HalfedgeMesh* mesh = geometry->getMesh();

    // Validate input parameters
    if (mesh == nullptr) {
        throw std::runtime_error("computeLocalFeatureSize_smooth: mesh is null");
    }

    size_t nVerts = mesh->nVertices();
    if (nVerts == 0) {
        throw std::runtime_error("computeLocalFeatureSize_smooth: mesh has no vertices");
    }

    // Validate Laplacian dimensions
    if (zeroFormLaplacian.nRows() != nVerts || zeroFormLaplacian.nColumns() != nVerts) {
        throw std::runtime_error("computeLocalFeatureSize_smooth: Laplacian size mismatch. Expected " +
                                 std::to_string(nVerts) + "x" + std::to_string(nVerts) +
                                 " but got " + std::to_string(zeroFormLaplacian.nRows()) +
                                 "x" + std::to_string(zeroFormLaplacian.nColumns()));
    }

    // Minimum feature size throughout the shape
    // TODO not the most principled value; orientation dependent
    double minCurvature = 1.0 / std::pow(geometry->lengthScale(), 2.0);
    VertexData<double> featureCurvature(mesh);

    for(VertexPtr v : mesh->vertices()) {
        
        // Compute pointwise curvature
        double ptwiseK = geometry->angleDefect(v) / geometry->area(v.dual());
        if(v.isBoundary()) {
            ptwiseK = 0.0; // TODO decide what to do, if anything, about boundary curvature
        }

        double ptwiseCurvatureMag = std::abs(ptwiseK);
        featureCurvature[v] = std::max(minCurvature, ptwiseCurvatureMag);
    }

    // Smooth
    // TODO Really, I think we would want to do something here that is like smoothing, but only increases the quantity from its
    // initial value.
    double smoothCoef = smoothing; // how much smoothing to do (1 --> 100% smoothing)
    GC::SparseMatrix<double> smoothOp = smoothCoef * zeroFormLaplacian + (1.0 - smoothCoef) * GC::SparseMatrix<double>::identity(mesh->nVertices());
    GC::DenseVector<double> rhs = (1.0 - smoothCoef) * featureCurvature.toVector();

    // Initialize result vector with correct size before solving
    GC::DenseVector<double> result(mesh->nVertices());

    // solvePositiveDefinite(smoothOp, result, rhs);
    GC::solve(smoothOp, result, rhs);

    VertexData<double> smoothedCurvature(mesh, result);
    VertexData<double> featureSize(mesh);
    for(VertexPtr v : mesh->vertices()) {
        featureSize[v] = 1.0 / std::sqrt(smoothedCurvature[v]);
    }

    return featureSize;

}
