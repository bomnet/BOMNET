//
// BOMNET PLUG-IN FILE
//


#include "inet/common/geometry/shape/polyhedron/IfcPolyhedron.h"

#include <algorithm>

#include "inet/common/stlutils.h"

namespace inet {

/*
 * Based on the following book chapter:
 * Mark de Berg, Marc van Kreveld, Mark Overmars, and Otfried Schwarzkopf (2008).
 * Computational Geometry (Third edition). Springer-Verlag.
 * 11.2 Computing Convex Hulls in 3-Space. Includes a description of the algorithm.
 */
void IfcPolyhedron::buildConvexHull()
{
    createInitialTetrahedron();
//    std::random_shuffle(points.begin(), points.end());
    initializeConflictGraph();
    for (auto currentPoint : points) {
        if (currentPoint->isSelected())
            continue;
        currentPoint->setToSelected();
        // If there is no conflicts then the point is already contained in the current convex hull
        if (currentPoint->hasConflicts()) {
            // Delete all faces in currentPoint conflict vector from the hull
            Faces& conflictVector = currentPoint->getConflictVector();
            Faces copyConflictVector = currentPoint->getConflictVector();
            purgeConflictFaces(conflictVector);
            // Computes a closed curve consisting of edges of the current convex hull
            // This closed curve can be interpreted as the border of the visible region from our current point
            Edges horizonEdges = computeHorizonEdges(conflictVector);
            for (auto horizonEdge : horizonEdges) {
                // Faces incident to horizonEdge in the current convex hull
                PolyhedronFace *neighborFace = horizonEdge->getJointFace();
                PolyhedronFace *parentFace = horizonEdge->getParentFace();
                // Connect horizonEdge to currentPoint by creating a triangular face newFace
                PolyhedronFace *newFace = new PolyhedronFace(horizonEdge->getP1(), horizonEdge->getP2(), currentPoint);
                // Coplanar faces have to be merged together since they define a bigger face
                // Due to these merges the computeIntersection() algorithm will have to visit fewer faces
                // It is clear that conflict list and outward normals are same as that of neighborFace
                connectFaces(newFace); // TODO optimize
                if (areCoplanar(newFace, neighborFace)) {
                    mergeFaces(newFace, neighborFace, currentPoint);
                    connectFaces(neighborFace);
                }
                else {
                    // If they are not coplanar we just add it to the faces and compute its outward normal vector
                    // and update the conflict graph
                    newFace->setOutwardNormalVector(computeOutwardNormalVector(newFace));
                    addFace(newFace);
                    setContlictListForNewFace(newFace, neighborFace, parentFace);
                }
            }
            // We have to delete the old faces that wrapped by the new faces
            cleanConflictGraph(copyConflictVector);
        }
    }
    purgeWrappedFaces();
}

bool IfcPolyhedron::areCollinear(const PolyhedronPoint *lineP1, const PolyhedronPoint *lineP2, const PolyhedronPoint *point) const
{
    PolyhedronPoint P1P2 = *lineP2 - *lineP1;
    PolyhedronPoint P1Point = *point - *lineP1;
    return P1P2 % P1Point == Coord(0, 0, 0);
}

bool IfcPolyhedron::areCoplanar(const PolyhedronPoint *p1, const PolyhedronPoint *p2, const PolyhedronPoint *p3, const PolyhedronPoint *p4) const
{
    // http://mathworld.wolfram.com/Coplanar.html
    // The volume of the tetrahedron they define.
    return (*p3 - *p1) * ((*p2 - *p1) % (*p4 - *p3)) == 0;
}

void IfcPolyhedron::createInitialTetrahedron()
{
    // Initially, we choose four points that do not lie in a common plane, so that their
    // convex hull is a tetrahedron.
    auto it = points.begin();
    PolyhedronPoint *p1 = *it;
    it++;
    p1->setToSelected();
    PolyhedronPoint *p2 = *it;
    p2->setToSelected();
    Points tetrahedronPoints;
    tetrahedronPoints.push_back(p1);
    tetrahedronPoints.push_back(p2);
    PolyhedronPoint *p3 = nullptr;
    it++;
    while (it != points.end() && !p3) {
        // Find the point that does not lie on the line through p1 and p2.
        if (!areCollinear(p1, p2, *it))
            p3 = *it;
        else
            it++;
    }
    if (!p3)
        throw cRuntimeError("All points lie on the same line");
    p3->setToSelected();
    tetrahedronPoints.push_back(p3);
    PolyhedronPoint *p4 = nullptr;
    it++;
    while (it != points.end() && !p4) {
        // Find the point that does not lie in the plane through p1, p2 and p3.
        if (!areCoplanar(p1, p2, p3, *it))
            p4 = *it;
        else
            it++;
    }
    if (!p4)
        throw cRuntimeError("All points lie in the same plane");
    tetrahedronPoints.push_back(p4);
    p4->setToSelected();
    generateAndAddTetrahedronFaces(tetrahedronPoints);
}

void IfcPolyhedron::generateAndAddTetrahedronFaces(const Points& tetrahedronPoints)
{
    // We just make CH({p1, p2, p3, p4}), note that, these points are that we have just selected
    // in crateInitialTetrahedron()
    PolyhedronFace *face1 = new PolyhedronFace(tetrahedronPoints[0], tetrahedronPoints[1], tetrahedronPoints[2]);
    PolyhedronFace *face2 = new PolyhedronFace(tetrahedronPoints[0], tetrahedronPoints[1], tetrahedronPoints[3]);
    PolyhedronFace *face3 = new PolyhedronFace(tetrahedronPoints[0], tetrahedronPoints[2], tetrahedronPoints[3]);
    PolyhedronFace *face4 = new PolyhedronFace(tetrahedronPoints[1], tetrahedronPoints[2], tetrahedronPoints[3]);
    // Add the faces to the convex hull
    addFace(face1);
    addFace(face2);
    addFace(face3);
    addFace(face4);
    connectFaces(face1);
    connectFaces(face2);
    connectFaces(face3);
    connectFaces(face4);
    // Calculate the outward normals
    face1->setOutwardNormalVector(computeOutwardNormalVector(face1));
    face2->setOutwardNormalVector(computeOutwardNormalVector(face2));
    face3->setOutwardNormalVector(computeOutwardNormalVector(face3));
    face4->setOutwardNormalVector(computeOutwardNormalVector(face4));
}

void IfcPolyhedron::addFace(PolyhedronFace *face)
{
    faces.push_back(face);
}

IfcPolyhedron::Edges IfcPolyhedron::computeHorizonEdges(const Faces& visibleFaces) const
{
    Edges horizonEdges;
    for (Faces::const_iterator it = visibleFaces.begin(); it != visibleFaces.end(); it++) {
        PolyhedronFace *visibleFace = *it;
        Edges& edges = visibleFace->getEdges();
        for (auto visibleEdge : edges) {
            // If the jointFace is not visible then we just have found a horizon edge, that is, a boundary edge
            // of the visible region from an arbitrary point.
            // If we found the jointFace among the visibleFaces then we are just examining an inner face.
            if (!contains(visibleFaces, visibleEdge->getJointFace()))
                horizonEdges.push_back(visibleEdge);
        }
    }
    return horizonEdges;
}

void IfcPolyhedron::initializeConflictGraph()
{
    for (auto point : points) {
        for (auto face : faces) {
            // The conflict graph is a bipartite graph with point class and face class
            if (face->isVisibleFrom(point)) {
                point->addConflictFace(face);
                face->addConflictPoint(point);
            }
        }
    }
}

bool IfcPolyhedron::areCoplanar(const PolyhedronFace *face1, const PolyhedronFace *face2) const
{
    Coord faceNormal1 = face1->getNormalVector();
    Coord faceNormal2 = face2->getNormalVector();
    return faceNormal1 % faceNormal2 == Coord(0, 0, 0);
}

void IfcPolyhedron::mergeFaces(PolyhedronFace *newFace, PolyhedronFace *neighborFace, PolyhedronPoint *point)
{
    Edges& edges = neighborFace->getEdges();
    auto eit = edges.begin();
    PolyhedronEdge *edge = *eit;
    // TODO optimize
    while (edge->getJointFace() != newFace) {
        eit++;
        if (eit == edges.end())
            throw cRuntimeError("Tried to merge two faces that do not share a common edge");
        edge = *eit;
    }
    // Delete this common edge from the edge vector, but keep its points, next and prev edges to create the merged
    // face
    eit = edges.erase(eit);
    PolyhedronPoint *p1 = edge->getP1();
    PolyhedronPoint *p2 = edge->getP2();
    // Create the new edges in neighborFace (we extend neighbor face with newFace (which is a triangular face) to
    // keep it simple.
    PolyhedronEdge *edge1 = new PolyhedronEdge(p1, point, neighborFace);
    PolyhedronEdge *edge2 = new PolyhedronEdge(point, p2, neighborFace);
    // We must keep the order
    edges.insert(eit, edge1);
    eit++;
    edges.insert(eit, edge2);
    // Finally, we have to delete the newFace (since we merged and no longer needed) and neighborFace's edge
    delete newFace;
    delete edge;
}

void IfcPolyhedron::setContlictListForNewFace(PolyhedronFace *newFace, const PolyhedronFace *neighbor1, const PolyhedronFace *neighbor2)
{
    // Test union of neighbor1's and neighbor2's conflict list
    std::map<PolyhedronPoint *, bool> visited;
    const Points& conflict1 = neighbor1->getConflictVector();
    const Points& conflict2 = neighbor2->getConflictVector();
    for (const auto& elem : conflict1) {
        PolyhedronPoint *point = elem;
        visited.insert(std::pair<PolyhedronPoint *, bool>(point, true));
        if (newFace->isVisibleFrom(elem)) {
            newFace->addConflictPoint(point);
            point->addConflictFace(newFace);
        }
    }
    for (auto point : conflict2) {
        if (!containsKey(visited, point) && newFace->isVisibleFrom(point)) {
            point->addConflictFace(newFace);
            newFace->addConflictPoint(point);
        }
    }
}

void IfcPolyhedron::cleanConflictGraph(const Faces& conflictVector)
{
    for (auto face : conflictVector) {
        Points& pConflict = face->getConflictVector();
        for (auto point : pConflict) {
            Faces& currFConflict = point->getConflictVector();
            auto fit2 = find(currFConflict, face);
            if (fit2 == currFConflict.end())
                throw cRuntimeError("PolyhedronFace not found in the point's conflict vector");
            currFConflict.erase(fit2);
        }
    }
}

IfcPolyhedron::IfcPolyhedron(const std::vector<Coord>& points, const std::vector<CustomFace>& customFaces)
{
    if (points.size() < 4)
        throw cRuntimeError("We need at least four points");
    for (const auto& point : points)
        this->points.push_back(new PolyhedronPoint(point));


    for (const auto& customFace : customFaces) {
                // Get points for the face from the custom face indices
                PolyhedronPoint *p1 = this->points.at(customFace.p1);
                PolyhedronPoint *p2 = this->points.at(customFace.p2);
                PolyhedronPoint *p3 = this->points.at(customFace.p3);
                // Create the face
                PolyhedronFace *face = new PolyhedronFace(p1, p2, p3);

                addFace(face);
                connectFaces(face);

                face->setOutwardNormalVector(face->getNormalVector());
    }
}

void IfcPolyhedron::purgeConflictFaces(const Faces& conflictVector)
{
    for (auto face : conflictVector)
        face->setToWrapped();
}

PolyhedronPoint IfcPolyhedron::computeOutwardNormalVector(const PolyhedronFace *face) const
{
    if (faces.size() <= 0)
        throw cRuntimeError("You can't compute the outward normal vector if you have no faces at all");
    PolyhedronFace *testFace = faces.at(0);
    if (face == testFace)
        testFace = faces.at(1);
    Coord testCentroid = testFace->getCentroid();
    PolyhedronPoint *facePoint = face->getEdge(0)->getP1();
    Coord faceNormal = face->getNormalVector();
    if ((testCentroid - *facePoint) * faceNormal > 0)
        return faceNormal * (-1);
    return faceNormal;
}

void IfcPolyhedron::connectFaces(PolyhedronFace *newFace)
{
    Edges& newEdges = newFace->getEdges();
    for (auto newEdge : newEdges) {
        for (auto currentFace : faces) {
            if (currentFace == newFace || currentFace->isWrapped()) continue;
            PolyhedronEdge *currEdge = currentFace->findEdge(newEdge);
            if (currEdge) {
                currEdge->setJointFace(newFace);
                newEdge->setJointFace(currentFace);
            }
        }
    }
}

void IfcPolyhedron::purgeWrappedFaces()
{
    auto fit = faces.begin();
    while (fit != faces.end()) {
        PolyhedronFace *face = *fit;
        if (face->isWrapped()) {
            fit = faces.erase(fit);
            delete face;
        }
        else
            fit++;
    }
}

Coord IfcPolyhedron::computeBoundingBoxSize() const
{
    Coord min;
    Coord max;
    for (const auto& elem : points) {
        Coord point = *(elem);
        min = min.min(point);
        max = max.max(point);
    }
    return max - min;
}

bool is_point_inside_triangle(const Coord& point, const Coord& A, const Coord& B, const Coord& C) {
    // Compute vectors
    Coord v0 = C - A;
    Coord v1 = B - A;
    Coord v2 = point - A;

    // Compute dot products
    double dot00 = v0 * v0;
    double dot01 = v0 * v1;
    double dot02 = v0 * v2;
    double dot11 = v1 * v1;
    double dot12 = v1 * v2;

    // Compute barycentric coordinates
    double inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // Check if point is in triangle
    return (u >= 0) && (v >= 0) && (u + v <= 1);
}


Coord line_plane_intersection(const Coord& P0, const Coord& P1, const Coord& segmentDirection, const Coord& normalVec, const Coord& centroid, const PolyhedronFace& face) {
    double a = normalVec.x, b = normalVec.y, c = normalVec.z;
    double d_plane = -(a * centroid.x + b * centroid.y + c * centroid.z); // Plane equation: ax + by + cz + d = 0

    double t = -(a * P0.x + b * P0.y + c * P0.z + d_plane) / (a * segmentDirection.x + b * segmentDirection.y + c * segmentDirection.z);

    Coord intersection_point = {P0.x + t * segmentDirection.x, P0.y + t * segmentDirection.y, P0.z + t * segmentDirection.z};

    if (t >= 0 && t <= 1) {
        Coord A = *(face.getEdge(0)->getP1());
        Coord B = *(face.getEdge(1)->getP1());
        Coord C = *(face.getEdge(2)->getP1());
        // Check if the intersection point lies within the bounds of the triangular face
        if (is_point_inside_triangle(intersection_point, A, B, C)) {
            return intersection_point;
        }
    }
    return Coord::NIL;
}

bool IfcPolyhedron::computeIntersection(const LineSegment& lineSegment, Coord& intersection1, Coord& intersection2, Coord& normal1, Coord& normal2) const
{
    Coord p0 = lineSegment.getPoint1();
    Coord p1 = lineSegment.getPoint2();
    if (p0 == p1) {
        normal1 = normal2 = Coord::NIL;
        return false;
    }
    Coord segmentDirection = p1 - p0;
    double min_dist_p0 = std::numeric_limits<double>::infinity();
    double min_dist_p1 = std::numeric_limits<double>::infinity();
    for (auto face : faces) {
        Coord normalVec = face->getOutwardNormalVector();
        Coord centroid = face->getCentroid();
        Coord intersection_point = line_plane_intersection(p0, p1, segmentDirection, normalVec, centroid, *face);
        if (intersection_point.x != 0 || intersection_point.y != 0 || intersection_point.z != 0) {
            double dist_p0 = sqrt(pow(intersection_point.x - p0.x, 2) + pow(intersection_point.y - p0.y, 2) + pow(intersection_point.z - p0.z, 2));
            double dist_p1 = sqrt(pow(intersection_point.x - p1.x, 2) + pow(intersection_point.y - p1.y, 2) + pow(intersection_point.z - p1.z, 2));

            if (dist_p0 < min_dist_p0) {
                intersection1 = intersection_point;
                min_dist_p0 = dist_p0;
            }

            if (dist_p1 < min_dist_p1) {
                intersection2 = intersection_point;
                min_dist_p1 = dist_p1;
            }
        }
    }
    if (intersection1 == intersection2) {
        normal1 = normal2 = Coord::NIL;
        return false;
    }
    return true;
}

void IfcPolyhedron::computeVisibleFaces(std::vector<std::vector<Coord>>& faces, const RotationMatrix& rotation, const RotationMatrix& viewRotation) const
{
    for (auto face : this->faces) {
        if (isVisibleFromView(face, viewRotation, rotation)) {
            const Edges& edges = face->getEdges();
            std::vector<Coord> points;
            for (auto edge : edges) {
                Coord point = *edge->getP1();
                points.push_back(point);
            }
            faces.push_back(points);
        }
    }
}

bool IfcPolyhedron::isVisibleFromView(const PolyhedronFace *face, const RotationMatrix& viewRotation, const RotationMatrix& rotation) const
{
    Coord zNormal(0, 0, 1);
    Coord rotatedFaceNormal = viewRotation.rotateVector(rotation.rotateVector(face->getOutwardNormalVector()));
    return rotatedFaceNormal * zNormal > 0;
}

IfcPolyhedron::~IfcPolyhedron()
{
    for (auto& elem : points)
        delete elem;
    for (auto& elem : faces)
        delete elem;
}

} /* namespace inet */

