#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {
  Vector2D lerp2D(Vector2D a, Vector2D b, double t) {
    return a + t * (b - a);
  }

  Vector3D lerp3D(Vector3D a, Vector3D b, double t) {
    return a + t * (b - a);
  }

  void BezierCurve::evaluateStep() {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.

    int numCurrentLevel = evaluatedLevels.size();
    vector<Vector2D> lastLevel = evaluatedLevels.back();

    if (lastLevel.size() > 1) {
      vector<Vector2D> currentLevel;
      for (int i = 0; i < lastLevel.size() - 1; i++)
          currentLevel.push_back(lerp2D(lastLevel[i], lastLevel[i+1], t));

      evaluatedLevels.push_back(currentLevel);
    }

    return;
  }


  Vector3D BezierPatch::evaluate(double u, double v) const {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
    std::vector<Vector3D> intermediatePoints;

    for(int i = 0; i < 4; i++)
      intermediatePoints.push_back(evaluate1D(controlPoints[i],u));
    
    return evaluate1D(intermediatePoints,v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
    vector<Vector3D> lastLevel = points;

    for (int i = 0; i < 3; ++i) {// 4 controlPoints 3 step evaluate
      vector<Vector3D> currentLevel;
      for (int i = 0; i < lastLevel.size() - 1; i++) // every step we lerp (numControlPoints-1) times
          currentLevel.push_back(lerp3D(lastLevel[i], lastLevel[i+1], t));

      lastLevel = currentLevel;
    }
    if (lastLevel.size() != 1)
      cout << "Bug appears in evaluate1D()!\n";
    return lastLevel[0];
  }



  Vector3D Vertex::normal( void ) const {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    
    Vector3D n(0, 0, 0); // initialize a vector to store your normal sum
    HalfedgeCIter h = halfedge(); // Since we're in a Vertex, this returns a halfedge pointing away from that vertex
    HalfedgeCIter h_orig = h;

    do {
      Vector3D a = this->position;
      Vector3D b = h->next()->vertex()->position;
      Vector3D c = h->next()->next()->vertex()->position;
      n += cross(b-a,c-a);//*cross(b-a,c-a).norm();
      h = h->twin()->next();
    } while (h != h_orig);

    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 ) {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->halfedge()->isBoundary()) return e0;

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();   
    
    // void setNeighbors( HalfedgeIter next,
    //                         HalfedgeIter twin,
    //                         VertexIter vertex,
    //                         EdgeIter edge,
    //                         FaceIter face )
    h0->setNeighbors(h5, h3, v2, e0, f0);
    h3->setNeighbors(h2, h0, v3, e0, f1);
    h5->setNeighbors(h1, h5->twin(), v3, h5->edge(), f0);
    h1->setNeighbors(h0, h1->twin(), v1, h1->edge(), f0);
    h2->setNeighbors(h4, h2->twin(), v2, h2->edge(), f1);
    h4->setNeighbors(h3, h4->twin(), v0, h4->edge(), f1);

    v0->halfedge() = h4;
    v1->halfedge() = h1;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 ) {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->halfedge()->isBoundary()) return e0->halfedge()->vertex();

    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();   
   
    HalfedgeIter h6 = newHalfedge();
    HalfedgeIter h7 = newHalfedge();
    HalfedgeIter h8 = newHalfedge();
    HalfedgeIter h9 = newHalfedge();
    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    EdgeIter e1 = newEdge();
    EdgeIter e2 = newEdge();
    EdgeIter e3 = newEdge();
    FaceIter f2 = newFace();
    FaceIter f3 = newFace();
    VertexIter mp = newVertex();

    mp->position = (v0->position + v1->position)/2.0;
    mp->halfedge() = h9;

    // void setNeighbors( HalfedgeIter next,
    //                         HalfedgeIter twin,
    //                         VertexIter vertex,
    //                         EdgeIter edge,
    //                         FaceIter face )
    h0->setNeighbors(h8, h9, v0, e0, f0);
    h3->setNeighbors(h10, h6, v1, e1, f3);
    h5->setNeighbors(h3, h5->twin(), v3, h5->edge(),f3);
    h1->setNeighbors(h7, h1->twin(), v1, h1->edge(),f1);
    h2->setNeighbors(h0, h2->twin(), v2, h2->edge(),f0);
    h4->setNeighbors(h11, h4->twin(), v0, h4->edge(),f2);
    h6->setNeighbors(h1, h3, mp, e1, f1);
    h7->setNeighbors(h6, h8, v2, e2, f1);
    h8->setNeighbors(h2, h7, mp, e2, f0);
    h9->setNeighbors(h4, h0, mp, e0, f2);
    h10->setNeighbors(h5, h11, mp, e3, f3);
    h11->setNeighbors(h9, h10, v3, e3, f2);

    f0->halfedge() = h0;
    f1->halfedge() = h6;
    f2->halfedge() = h9;
    f3->halfedge() = h3;

    e1->halfedge() = h6;
    e2->halfedge() = h7;
    e3->halfedge() = h10;

    e0->isNew = false;
    e1->isNew = false;
    e2->isNew = true;
    e3->isNew = true;
    mp->isNew = true;

    return mp;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh ) {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)


    // TODO Now flip any new edge that connects an old and new vertex.


    // TODO Finally, copy the new vertex positions into final Vertex::position.

    // 对每个原有点，根据围绕它的点的坐标，计算出新的位置，但暂不更新
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->isNew = false;

      float n = v->degree();
      float u = 3.0/(8.0*n);

      if (n == 3) u = 3.0/16.0;

      Vector3D around_v(0,0,0);

      // 围着点转一圈
      HalfedgeCIter e_orig = v->halfedge();
      HalfedgeCIter e = e_orig;
      do {
        around_v += e->twin()->vertex()->position;
        e = e->twin()->next();
      } while (e != e_orig);

      v->newPosition = (1-n*u) * v->position + u*around_v;
    }

    // 对每条原有边，根据围绕它的四个点，计算出将来split在原边上产生的新点的位置
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      HalfedgeIter eH = e->halfedge();
      Vector3D a = eH->vertex()->position;
      Vector3D b = eH->twin()->vertex()->position;
      Vector3D c = eH->next()->next()->vertex()->position;
      Vector3D d = eH->twin()->next()->next()->vertex()->position;

      e->newPosition = 3.0/8.0 * (a + b) + 1.0/8.0 * (c + d);
      e->isNew = false;
    }

    // 对每条原有的边进行 split，注意新产生的边不 split，并把刚才计算出的新点位置赋给新点
    EdgeIter e = mesh.edgesBegin();
    int numEdges = mesh.nEdges();
    for (int i = 0; i < numEdges; i++, ++e) {
      if(e->isNew == false){
        VertexIter v = mesh.splitEdge(e);
        v->isNew = true;
        v->position = e->newPosition;
      }
    }


    // 遍历所有新边，如果它连接老点和新点，则对它 flip
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (e->isNew) {
        HalfedgeIter h = e->halfedge();
        if ( (h->vertex()->isNew && !h->twin()->vertex()->isNew) || (!h->vertex()->isNew && h->twin()->vertex()->isNew) ) {
          e = mesh.flipEdge(e);
          e->isNew = false;    
        }
      }
    }

    // 将开始计算出的老点的新位置赋给老点
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      if (v->isNew == false) v->position = v->newPosition;
    }

    return;
  
  }
}
