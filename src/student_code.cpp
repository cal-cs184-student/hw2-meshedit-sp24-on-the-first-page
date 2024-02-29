#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
      int pointCount = points.size();
      std::vector<Vector2D> nextPoints;
      if (pointCount == 1) {
          //base 
          return points;
      }

      for (int i = 0; i < pointCount - 1; i++) {
          //recurse via linear interpolation
          int j = i + 1;
          nextPoints.push_back((1 - t) * points[i] + t * points[j]);
      }
      return nextPoints;
      //return std::vector<Vector2D>();
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      int pointCount = points.size();
      std::vector<Vector3D> nextPoints;
      if (pointCount == 1) {
          //base 
          return points;
      }

      for (int i = 0; i < pointCount - 1; i++) {
          //recurse via linear interpolation
          int j = i + 1;
          nextPoints.push_back((1 - t) * points[i] + t * points[j]);
      }
      return nextPoints;
    
    //return std::vector<Vector3D>();
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      int pointCount = points.size();
      if (pointCount == 1) {
          //base 
          return points[0];
      }
      return evaluate1D(evaluateStep(points, t), t);
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
      int pointCount = controlPoints.size();
      std::vector<Vector3D> nexPoints;
      for (int i = 0; i < pointCount; i++){
          nexPoints.push_back(evaluate1D(controlPoints[i],u));
      }
      return evaluate1D(nexPoints, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

      Vector3D finalVec(0, 0, 0);
      HalfedgeCIter half = halfedge();

      if (!half->face()->isBoundary()) {
          Vector3D pos1 = position;
          Vector3D pos2 = half->next()->vertex()->position;
          Vector3D pos3 = half->next()->next()->vertex()->position;

          Vector3D face = half->face()->normal();
          double triangleArea = 0.5 * (cross(pos2 - pos1, pos3 - pos1).norm());
          finalVec += (triangleArea * face);
      }
      half = half->twin()->next();

      while (half != halfedge()) {
          if (!half->face()->isBoundary()) {
              Vector3D pos1 = position;
              Vector3D pos2 = half->next()->vertex()->position;
              Vector3D pos3 = half->next()->next()->vertex()->position;

              Vector3D face = half->face()->normal();
              double triangleArea = 0.5 * (cross(pos2 - pos1, pos3 - pos1).norm());
              finalVec += (triangleArea * face);
          }

          half = half->twin()->next();
      }

    return finalVec.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.

     //STEP 1 Define Half Edges
     //reg edges are triangle on left side, flip edges are on left side. In means inner, Out means outer
     // do picture of this for website

     HalfedgeIter reg1in = e0->halfedge();
     HalfedgeIter flip1in = reg1in->twin();
     HalfedgeIter reg2in = reg1in->next();
     HalfedgeIter flip2in = flip1in->next();
     HalfedgeIter reg3in = reg2in->next();
     HalfedgeIter flip3in = flip2in->next();

     HalfedgeIter reg2out = reg2in->twin();
     HalfedgeIter flip2out = flip2in->twin();
     HalfedgeIter reg3out = reg3in->twin();
     HalfedgeIter flip3out = flip3in->twin();

     //STEP 2 Define Edges, Vertices, and Faces
     EdgeIter edge1reg = reg1in->edge();
     EdgeIter edge2reg = reg2in->edge();
     EdgeIter edge3reg = reg3in->edge();
     EdgeIter edge2flip = flip2in->edge();
     EdgeIter edge3flip = flip3in->edge();

     VertexIter northVertex = reg1in->vertex();
     VertexIter eastVertex = flip3in->vertex();
     VertexIter southVertex = flip1in->vertex();
     VertexIter westVertex = reg3in->vertex();
     //Iterated like a compass. Relying heavily on images from Lecture 8 Slide 24
     //No longer sure; North and South may be switched...

     FaceIter facereg = reg1in->face();
     FaceIter faceflip = flip1in->face();

     //STEP NOW FLIP = REG AND REG = FLIP!
     //except whoops, not quite. True for Vertices, but for edges it's more of a "move 1 down the line" sort of deal (reg2->reg3 and reg3->flip2)
     //and for faces I'm not sure? just try both every time. 

     reg1in->setNeighbors(reg2in, flip1in, westVertex, edge1reg, facereg);
     reg2in->setNeighbors(reg3in, flip3out, eastVertex, edge3flip, facereg);
     reg3in->setNeighbors(reg1in, reg2out, southVertex, edge2reg, facereg);
     flip1in->setNeighbors(flip2in, reg1in, eastVertex, edge1reg, faceflip);
     flip2in->setNeighbors(flip3in, reg3out, westVertex, edge3reg, faceflip);
     flip3in->setNeighbors(flip1in, flip2out, northVertex, edge2flip, faceflip);

     FaceIter faceReg2= reg2out->face();
     FaceIter faceReg3 = reg3out->face();
     FaceIter faceFlip2 = flip2out->face();
     FaceIter faceFlip3 = flip3out->face();

     reg2out->setNeighbors(reg2out->next(), reg3in, westVertex, edge2reg, faceReg2);
     reg3out->setNeighbors(reg3out->next(), flip2in, northVertex, edge3reg, faceReg3);
     flip2out->setNeighbors(flip2out->next(), flip3in, eastVertex, edge2flip, faceFlip2);
     flip3out ->setNeighbors(flip3out->next(), reg2in, southVertex, edge3flip, faceFlip3);

     edge1reg->halfedge() = reg1in;
     edge2reg->halfedge() = reg3in;
     edge3reg->halfedge() = flip2in;
     edge2flip->halfedge() = flip3in;
     edge3flip->halfedge() = reg2in;

     northVertex->halfedge() = flip3in;
     southVertex->halfedge() = reg3in;
     westVertex->halfedge() = flip2in;
     eastVertex->halfedge() = reg2in;

     facereg->halfedge() = reg1in;
     faceflip->halfedge() = flip1in;

    return edge1reg;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

      //STEP 1 Define Half Edges
     //reg edges are triangle on left side, flip edges are on left side. In means inner, Out means outer
     // do picture of this for website

      HalfedgeIter reg1in = e0->halfedge();
      HalfedgeIter flip1in = reg1in->twin();
      HalfedgeIter reg2in = reg1in->next();
      HalfedgeIter flip2in = flip1in->next();
      HalfedgeIter reg3in = reg2in->next();
      HalfedgeIter flip3in = flip2in->next();

      HalfedgeIter reg2out = reg2in->twin();
      HalfedgeIter flip2out = flip2in->twin();
      HalfedgeIter reg3out = reg3in->twin();
      HalfedgeIter flip3out = flip3in->twin();

      //STEP 2 Define Edges, Vertices, and Faces
      EdgeIter edge1reg = reg1in->edge();
      EdgeIter edge2reg = reg2in->edge();
      EdgeIter edge3reg = reg3in->edge();
      EdgeIter edge2flip = flip2in->edge();
      EdgeIter edge3flip = flip3in->edge();

      VertexIter northVertex = reg1in->vertex();
      VertexIter eastVertex = flip3in->vertex();
      VertexIter southVertex = flip1in->vertex();
      VertexIter westVertex = reg3in->vertex();
      //Iterated like a compass. Relying heavily on images from Lecture 8 Slide 24
      //No longer sure; North and South may be switched...

      FaceIter faceReg = reg1in->face();
      FaceIter faceFlip = flip1in->face();

      //STEP 3 Define new parts for split
      HalfedgeIter mtoabot = newHalfedge();
      //HalfedgeIter mtoatop = newHalfedge();
      //HalfedgeIter mtodtop = newHalfedge();
      HalfedgeIter mtodbot = newHalfedge();
      HalfedgeIter mtobreg = newHalfedge();
      HalfedgeIter mtobflip = newHalfedge();
      HalfedgeIter trueRegBot = newHalfedge();
      HalfedgeIter trueFlipBot = newHalfedge();


      EdgeIter mtoa = newEdge();
      EdgeIter mtob = newEdge();
      EdgeIter mtod = newEdge();

      VertexIter splitVertex = newVertex(); //vertex M on image

      FaceIter botFaceReg = newFace();
      FaceIter botFaceFlip= newFace();

      //STEP 4 SPLIT!

      reg1in->setNeighbors(reg2in, flip1in, splitVertex, edge1reg, faceReg);
      reg2in->setNeighbors(reg3in, reg2out, southVertex, edge2reg, faceReg);
      reg3in->setNeighbors(reg1in, mtoabot, westVertex, mtoa, faceReg); //weirdedge, actually mtoa b/c of new triangle

      flip1in->setNeighbors(flip2in, reg1in, southVertex, edge1reg, faceFlip);
      flip2in->setNeighbors(flip3in, mtodbot, splitVertex, mtod, faceFlip); //WEIRD EDGE, actually mtod b/c of new triangle
      flip3in->setNeighbors(flip1in, flip3out, eastVertex, edge3flip, faceFlip);

      FaceIter faceReg2 = reg2out->face();
      FaceIter faceReg3 = reg3out->face();
      FaceIter faceFlip2 = flip2out->face();
      FaceIter faceFlip3 = flip3out->face();

      reg2out->setNeighbors(reg2out->next(), reg2in, westVertex, edge2reg, faceReg2);
      reg3out->setNeighbors(reg3out->next(), trueRegBot, northVertex, edge3reg, faceReg3);

      flip2out->setNeighbors(flip2out->next(), trueFlipBot, eastVertex, edge2flip, faceFlip2);
      flip3out->setNeighbors(flip3out->next(), flip3in, southVertex, edge3flip, faceFlip3);

      mtoabot->setNeighbors(trueRegBot, reg3in, splitVertex, mtoa, botFaceReg);
      //mtoatop has become reg3in

      mtobreg->setNeighbors(mtoabot, mtobflip, northVertex, mtob, botFaceReg); 
      mtobflip->setNeighbors(trueFlipBot, mtobreg, splitVertex, mtob, botFaceFlip);

      mtodbot->setNeighbors(mtobflip, flip2in, eastVertex, mtod, botFaceFlip);
      //mtoatop has become flip2in

      trueRegBot->setNeighbors(mtobreg, reg3out, westVertex, edge3reg, botFaceReg);
      trueFlipBot->setNeighbors(mtodbot, flip2out, northVertex, edge2flip, botFaceFlip);

      edge1reg->halfedge() = reg1in;
      edge2reg->halfedge() = reg2in;
      edge3reg->halfedge() = trueRegBot;
      edge2flip->halfedge() = trueFlipBot;
      edge3flip->halfedge() = flip3in;
      mtoa -> halfedge() = reg3in;
      mtob->halfedge() = mtobreg;
      mtod->halfedge() = flip2in;

      mtoa->isNew = 1;
      mtod->isNew = 1;
      edge1reg->isNew = 0;
      mtob->isNew = 0;

      splitVertex -> position = 0.5 * (northVertex->position + southVertex->position); //split is midpoint
      splitVertex->isNew = 1;

      northVertex->halfedge() = mtobreg;
      southVertex->halfedge() = reg2in;
      westVertex->halfedge() = trueRegBot;
      eastVertex->halfedge() = flip3in;
      splitVertex->halfedge() = reg1in;

      faceReg->halfedge() = reg1in;
      faceFlip->halfedge() = flip1in;
      botFaceReg->halfedge() = mtobreg;
      botFaceFlip->halfedge() = mtobflip;

      return splitVertex;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.
      for (EdgeIter edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {

          //data
          edge->isNew = 0;

          HalfedgeIter reg1in = edge->halfedge();
          HalfedgeIter flip1in = reg1in->twin();
          HalfedgeIter reg2in = reg1in->next();
          HalfedgeIter flip2in = flip1in->next();
          HalfedgeIter reg3in = reg2in->next();
          HalfedgeIter flip3in = flip2in->next();

          VertexIter northVertex = reg1in->vertex();
          VertexIter eastVertex = flip3in->vertex();
          VertexIter southVertex = flip1in->vertex();
          VertexIter westVertex = reg3in->vertex();

          // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
          // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
          // a vertex of the original mesh.

          edge->newPosition = (3.0 / 8.0) * (northVertex->position + southVertex->position) + (1.0 / 8.0) * (westVertex->position + eastVertex->position);

      }
      
        
      // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      for (VertexIter vertex = mesh.verticesBegin(); vertex != mesh.verticesEnd(); vertex++) {
          vertex->isNew = 0;
          Vector3D original_neighbor_position(0, 0, 0);

          original_neighbor_position += vertex->halfedge()->twin()->vertex()->position;
          HalfedgeIter half = vertex->halfedge()->twin()->next();
          while (half != vertex->halfedge()) {
              original_neighbor_position += half->twin()->vertex()->position;
              half = half->twin()->next();
          }
          float n = (float) vertex->degree();
          float u = (3.0 / (8.0 * n));
          if (n == 3) {
              u = (3.0 / 16.0);
          }
          
          Vector3D original_position = vertex->position;
          vertex->newPosition = (1.0 - (n * u)) * original_position + u * original_neighbor_position;
          //the equation
      }

    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
      for (EdgeIter edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {
          //both points on either side of edge
          VertexIter endpoint1 = edge->halfedge()->vertex();
          VertexIter endpoint2 = edge->halfedge()->twin()->vertex();

          //split if og edge
          if (!endpoint1->isNew && !endpoint2->isNew) {
              VertexIter vertex = mesh.splitEdge(edge);
              vertex->newPosition = edge->newPosition;
          }
      }

    // 4. Flip any new edge that connects an old and new vertex.

      for (EdgeIter edge = mesh.edgesBegin(); edge != mesh.edgesEnd(); edge++) {
          //both points on either side of edge
          VertexIter endpoint1 = edge->halfedge()->vertex();
          VertexIter endpoint2 = edge->halfedge()->twin()->vertex();

          if (edge->isNew && ((endpoint1->isNew && !endpoint2->isNew) || (!endpoint1->isNew && endpoint2->isNew))) {
              mesh.flipEdge(edge);
          }
      }

    // 5. Copy the new vertex positions into final Vertex::position.

      for (VertexIter vertex = mesh.verticesBegin(); vertex != mesh.verticesEnd(); vertex++) {
          vertex->position = vertex->newPosition;
          vertex->isNew = 0;
          //not new and newPosition = true position
      }

  }
}
