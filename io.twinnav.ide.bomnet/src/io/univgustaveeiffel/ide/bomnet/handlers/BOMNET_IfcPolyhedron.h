//
// BOMNET PLUG-IN FILE
//

#ifndef __INET_IFCPOLYHEDRON_H
#define __INET_IFCPOLYHEDRON_H

#include "inet/common/geometry/base/ShapeBase.h"
#include "inet/common/geometry/common/RotationMatrix.h"
#include "inet/common/geometry/shape/polyhedron/PolyhedronEdge.h"
#include "inet/common/geometry/shape/polyhedron/PolyhedronFace.h"
#include "inet/common/geometry/shape/polyhedron/PolyhedronPoint.h"

namespace inet {

struct CustomFace {
    int p1;
    int p2;
    int p3;
};
/*
 * This class represents a convex polyhedron.
 * It takes a 3D point set and builds its convex hull.
 */
class INET_API IfcPolyhedron: public ShapeBase
{
  public:
    typedef std::vector<PolyhedronPoint *> Points;
    typedef std::vector<PolyhedronFace *> Faces;
    typedef std::vector<PolyhedronEdge *> Edges;

  protected:
    Faces faces;
    Points points;

  protected:
    void purgeWrappedFaces();
    void buildConvexHull();
    bool areCollinear(const PolyhedronPoint *lineP1, const PolyhedronPoint *lineP2, const PolyhedronPoint *point) const;
    bool areCoplanar(const PolyhedronPoint *p1, const PolyhedronPoint *p2, const PolyhedronPoint *p3, const PolyhedronPoint *p4) const;
    bool areCoplanar(const PolyhedronFace *face1, const PolyhedronFace *face2) const;
    void mergeFaces(PolyhedronFace *newFace, PolyhedronFace *neighborFace, PolyhedronPoint *point);
    void createInitialTetrahedron();
    void initializeConflictGraph();
    void cleanConflictGraph(const Faces& conflictVector);
    void purgeConflictFaces(const Faces& conflictVector);
    void connectFaces(PolyhedronFace *newFace);
    void setContlictListForNewFace(PolyhedronFace *newFace, const PolyhedronFace *neighbor1, const PolyhedronFace *neighbor2);
    void generateAndAddTetrahedronFaces(const Points& tetrahedronPoints);
    PolyhedronPoint computeOutwardNormalVector(const PolyhedronFace *face) const;
    void addFace(PolyhedronFace *face);
    Edges computeHorizonEdges(const Faces& visibleFaces) const;
    bool isVisibleFromView(const PolyhedronFace *face, const RotationMatrix& viewRotation, const RotationMatrix& rotation) const;

  public:
    IfcPolyhedron(const std::vector<Coord>& points, const std::vector<CustomFace>& customFaces);
    Coord computeBoundingBoxSize() const override;
    void computeVisibleFaces(std::vector<std::vector<Coord>>& faces, const RotationMatrix& rotation, const RotationMatrix& viewRotation) const;
    bool computeIntersection(const LineSegment& lineSegment, Coord& intersection1, Coord& intersection2, Coord& normal1, Coord& normal2) const override;
    const Faces& getFaces() const { return faces; }
    const Points& getPoints() const { return points; }
    virtual ~IfcPolyhedron();
};

} /* namespace inet */

#endif

