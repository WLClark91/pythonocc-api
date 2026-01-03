# main.py - Complete pythonOCC API for STEP file analysis with mesh generation
# Updated with proper hole vs boss detection based on surface normal direction

import base64
import io
import tempfile
import os
import uuid
from typing import Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# pythonOCC imports
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_REVERSED
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.GProp import GProp_GProps
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCC.Core.GeomAbs import (
    GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Torus, 
    GeomAbs_Cone, GeomAbs_Circle
)
from OCC.Core.gp import gp_Pnt, gp_Vec
from OCC.Core.BRepLProp import BRepLProp_SLProps
from OCC.Core.BRepTools import breptools

app = FastAPI(title="PythonOCC STEP Analyzer API")

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class AnalyzeRequest(BaseModel):
    fileBase64: str
    fileName: str

class FeatureLocation(BaseModel):
    x: float
    y: float
    z: float

class FeatureMetadata(BaseModel):
    faceIds: Optional[list] = None
    confidence: Optional[float] = None
    detectionMethod: Optional[str] = None
    normalDirection: Optional[str] = None  # Added to track normal direction

class ExtractedFeature(BaseModel):
    type: str
    parameters: dict
    location: Optional[FeatureLocation] = None
    count: Optional[int] = 1
    id: Optional[str] = None
    metadata: Optional[FeatureMetadata] = None

class UnifiedMeshData(BaseModel):
    vertices: list
    indices: list
    normals: Optional[list] = None
    triangleFeatureMap: list
    boundingBox: dict

class AnalyzeResponse(BaseModel):
    success: bool
    features: Optional[list] = None
    dimensions: Optional[dict] = None
    volume: Optional[float] = None
    surfaceArea: Optional[float] = None
    meshData: Optional[UnifiedMeshData] = None
    error: Optional[str] = None


def read_step_file(file_content: bytes) -> 'TopoDS_Shape':
    """Read STEP file from bytes and return the shape."""
    with tempfile.NamedTemporaryFile(suffix='.step', delete=False) as tmp:
        tmp.write(file_content)
        tmp_path = tmp.name
    
    try:
        reader = STEPControl_Reader()
        status = reader.ReadFile(tmp_path)
        
        if status != IFSelect_RetDone:
            raise Exception("Failed to read STEP file")
        
        reader.TransferRoots()
        shape = reader.OneShape()
        return shape
    finally:
        os.unlink(tmp_path)


def get_bounding_box(shape) -> dict:
    """Calculate bounding box of the shape."""
    bbox = Bnd_Box()
    brepbndlib.Add(shape, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return {
        "min": {"x": xmin, "y": ymin, "z": zmin},
        "max": {"x": xmax, "y": ymax, "z": zmax}
    }


def get_dimensions(shape) -> dict:
    """Calculate dimensions from bounding box."""
    bbox = get_bounding_box(shape)
    return {
        "length": abs(bbox["max"]["x"] - bbox["min"]["x"]),
        "width": abs(bbox["max"]["y"] - bbox["min"]["y"]),
        "height": abs(bbox["max"]["z"] - bbox["min"]["z"])
    }


def get_volume_and_area(shape) -> tuple:
    """Calculate volume and surface area."""
    props = GProp_GProps()
    brepgprop.VolumeProperties(shape, props)
    volume = props.Mass()
    
    brepgprop.SurfaceProperties(shape, props)
    surface_area = props.Mass()
    
    return volume, surface_area


def get_face_centroid(face) -> FeatureLocation:
    """Get the centroid of a face."""
    props = GProp_GProps()
    brepgprop.SurfaceProperties(face, props)
    center = props.CentreOfMass()
    return FeatureLocation(x=center.X(), y=center.Y(), z=center.Z())


def is_cylindrical_face_a_boss(face, adaptor) -> tuple:
    """
    Determine if a cylindrical face is a BOSS or HOLE by analyzing surface normal direction.
    
    Algorithm:
    1. Get the cylinder's axis direction and location
    2. Sample a point on the face surface
    3. Get the surface normal at that point
    4. Calculate the radial direction (from axis to surface point)
    5. Compare the normal direction with the radial direction
    6. If normal points TOWARD axis center → HOLE (internal cavity)
    7. If normal points AWAY from axis center → BOSS (external protrusion)
    
    Returns:
        tuple: (is_boss: bool, confidence: float, normal_direction: str)
    """
    try:
        cylinder = adaptor.Cylinder()
        axis = cylinder.Axis()
        axis_dir = axis.Direction()
        axis_loc = axis.Location()
        
        # Get UV bounds of the face
        u_min, u_max, v_min, v_max = breptools.UVBounds(face)
        
        # Sample point at the middle of the face
        u_mid = (u_min + u_max) / 2.0
        v_mid = (v_min + v_max) / 2.0
        
        # Get point on surface
        surface_pnt = adaptor.Value(u_mid, v_mid)
        
        # Get surface normal at this point using BRepLProp_SLProps
        props = BRepLProp_SLProps(adaptor, u_mid, v_mid, 1, 1e-6)
        
        if not props.IsNormalDefined():
            # Fallback: if normal not defined, use heuristic based on radius
            return (False, 0.5, "undefined")
        
        normal = props.Normal()
        
        # Account for face orientation (REVERSED means normal is flipped)
        if face.Orientation() == TopAbs_REVERSED:
            normal.Reverse()
        
        # Calculate radial direction: from axis to surface point
        # Project point onto axis to find closest point on axis
        point_vec = gp_Vec(axis_loc, surface_pnt)
        axis_vec = gp_Vec(axis_dir)
        
        # Projection length along axis
        proj_length = point_vec.Dot(axis_vec)
        
        # Closest point on axis
        axis_point = gp_Pnt(
            axis_loc.X() + axis_dir.X() * proj_length,
            axis_loc.Y() + axis_dir.Y() * proj_length,
            axis_loc.Z() + axis_dir.Z() * proj_length
        )
        
        # Radial direction: from axis to surface point
        radial_vec = gp_Vec(axis_point, surface_pnt)
        if radial_vec.Magnitude() > 1e-10:
            radial_vec.Normalize()
        else:
            # Point is on the axis, can't determine
            return (False, 0.5, "on_axis")
        
        # Compare normal with radial direction
        # Positive dot product = normal points outward (BOSS)
        # Negative dot product = normal points inward (HOLE)
        normal_vec = gp_Vec(normal.X(), normal.Y(), normal.Z())
        dot_product = normal_vec.Dot(radial_vec)
        
        # Determine feature type based on normal direction
        is_boss = dot_product > 0
        
        # Confidence is based on how clearly the normal points in/out
        confidence = min(0.95, 0.7 + abs(dot_product) * 0.25)
        
        normal_direction = "outward" if is_boss else "inward"
        
        return (is_boss, confidence, normal_direction)
        
    except Exception as e:
        print(f"Error determining boss/hole: {e}")
        return (False, 0.5, "error")


def analyze_face_for_feature(face, face_id: str) -> Optional[ExtractedFeature]:
    """Analyze a single face to detect if it's part of a feature."""
    try:
        adaptor = BRepAdaptor_Surface(face)
        surface_type = adaptor.GetType()
        
        # Cylindrical face -> could be hole or boss
        if surface_type == GeomAbs_Cylinder:
            cylinder = adaptor.Cylinder()
            radius = cylinder.Radius()
            
            # Estimate depth/height from face bounds
            umin, umax, vmin, vmax = adaptor.FirstUParameter(), adaptor.LastUParameter(), \
                                      adaptor.FirstVParameter(), adaptor.LastVParameter()
            height_or_depth = abs(vmax - vmin)
            
            # Use surface normal analysis to determine hole vs boss
            is_boss, confidence, normal_direction = is_cylindrical_face_a_boss(face, adaptor)
            
            if is_boss:
                feature_type = "boss"
                params = {"diameter": radius * 2, "height": height_or_depth}
            else:
                feature_type = "hole"
                params = {"diameter": radius * 2, "depth": height_or_depth}
            
            return ExtractedFeature(
                type=feature_type,
                parameters=params,
                location=get_face_centroid(face),
                id=str(uuid.uuid4()),
                metadata=FeatureMetadata(
                    faceIds=[face_id],
                    confidence=confidence,
                    detectionMethod="cylindrical_face_normal_analysis",
                    normalDirection=normal_direction
                )
            )
        
        # Toroidal face -> fillet
        elif surface_type == GeomAbs_Torus:
            torus = adaptor.Torus()
            minor_radius = torus.MinorRadius()
            
            return ExtractedFeature(
                type="fillet",
                parameters={"radius": minor_radius},
                location=get_face_centroid(face),
                id=str(uuid.uuid4()),
                metadata=FeatureMetadata(
                    faceIds=[face_id],
                    confidence=0.80,
                    detectionMethod="toroidal_face"
                )
            )
        
        # Conical face -> chamfer
        elif surface_type == GeomAbs_Cone:
            cone = adaptor.Cone()
            angle = cone.SemiAngle()
            
            return ExtractedFeature(
                type="chamfer",
                parameters={"angle": abs(angle) * 57.2958, "width": 2.0},  # Convert to degrees
                location=get_face_centroid(face),
                id=str(uuid.uuid4()),
                metadata=FeatureMetadata(
                    faceIds=[face_id],
                    confidence=0.75,
                    detectionMethod="conical_face"
                )
            )
        
        # Planar faces could be pockets or slots (need more context)
        elif surface_type == GeomAbs_Plane:
            # Only flag as pocket if it's an internal planar face
            # This is a simplified heuristic
            props = GProp_GProps()
            brepgprop.SurfaceProperties(face, props)
            area = props.Mass()
            
            # Small planar faces might be pocket bottoms
            if area < 1000:  # Threshold in mm²
                return ExtractedFeature(
                    type="pocket",
                    parameters={"depth": 5.0, "area": area},  # Depth is estimated
                    location=get_face_centroid(face),
                    id=str(uuid.uuid4()),
                    metadata=FeatureMetadata(
                        faceIds=[face_id],
                        confidence=0.60,
                        detectionMethod="planar_face"
                    )
                )
        
        return None
        
    except Exception as e:
        print(f"Error analyzing face {face_id}: {e}")
        return None


def extract_features(shape) -> list:
    """Extract all features from the shape."""
    features = []
    face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_index = 0
    
    while face_explorer.More():
        face = face_explorer.Current()
        face_id = f"face_{face_index}"
        
        feature = analyze_face_for_feature(face, face_id)
        if feature:
            features.append(feature)
        
        face_explorer.Next()
        face_index += 1
    
    # Consolidate similar features (e.g., multiple hole faces -> one hole with count)
    consolidated = consolidate_features(features)
    return consolidated


def consolidate_features(features: list) -> list:
    """Consolidate similar features and count them."""
    consolidated = {}
    
    for feature in features:
        # Create a key based on type and approximate parameters
        key = f"{feature.type}"
        if "diameter" in feature.parameters:
            key += f"_d{round(feature.parameters['diameter'], 1)}"
        if "radius" in feature.parameters:
            key += f"_r{round(feature.parameters['radius'], 1)}"
        
        if key in consolidated:
            consolidated[key].count = (consolidated[key].count or 1) + 1
            # Aggregate face IDs
            if consolidated[key].metadata and feature.metadata:
                consolidated[key].metadata.faceIds.extend(feature.metadata.faceIds or [])
        else:
            consolidated[key] = feature
    
    return list(consolidated.values())


def generate_mesh_data(shape, features: list) -> UnifiedMeshData:
    """
    Generate triangulated mesh with feature mapping.
    This is the critical function that creates triangleFeatureMap.
    """
    # Tessellate the shape
    linear_deflection = 0.5  # Controls mesh density (smaller = finer)
    angular_deflection = 0.5
    mesh = BRepMesh_IncrementalMesh(shape, linear_deflection, False, angular_deflection, True)
    mesh.Perform()
    
    if not mesh.IsDone():
        raise Exception("Mesh generation failed")
    
    # Build face-to-feature mapping
    face_to_feature_index = {}
    for feat_idx, feature in enumerate(features):
        if feature.metadata and feature.metadata.faceIds:
            for face_id in feature.metadata.faceIds:
                face_to_feature_index[face_id] = feat_idx
    
    # Collect all triangles from all faces
    all_vertices = []
    all_indices = []
    all_normals = []
    triangle_feature_map = []
    
    vertex_offset = 0
    face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_index = 0
    
    while face_explorer.More():
        face = face_explorer.Current()
        face_id = f"face_{face_index}"
        
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation(face, location)
        
        if triangulation is not None:
            # Get transformation matrix
            trsf = location.Transformation()
            
            # Extract vertices
            nb_nodes = triangulation.NbNodes()
            for i in range(1, nb_nodes + 1):
                node = triangulation.Node(i)
                # Apply transformation
                transformed = node.Transformed(trsf)
                all_vertices.extend([transformed.X(), transformed.Y(), transformed.Z()])
                # Placeholder normals (could compute from triangles)
                all_normals.extend([0.0, 0.0, 1.0])
            
            # Extract triangles
            nb_triangles = triangulation.NbTriangles()
            for i in range(1, nb_triangles + 1):
                tri = triangulation.Triangle(i)
                n1, n2, n3 = tri.Get()
                
                # Adjust indices for global vertex array (0-indexed)
                all_indices.extend([
                    vertex_offset + n1 - 1,
                    vertex_offset + n2 - 1,
                    vertex_offset + n3 - 1
                ])
                
                # Map this triangle to its feature (or -1 if no feature)
                feature_idx = face_to_feature_index.get(face_id, -1)
                triangle_feature_map.append(feature_idx)
            
            vertex_offset += nb_nodes
        
        face_explorer.Next()
        face_index += 1
    
    # Get bounding box
    bbox = get_bounding_box(shape)
    
    return UnifiedMeshData(
        vertices=all_vertices,
        indices=all_indices,
        normals=all_normals,
        triangleFeatureMap=triangle_feature_map,
        boundingBox=bbox
    )


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy", "service": "pythonocc-api"}


@app.post("/analyze", response_model=AnalyzeResponse)
async def analyze_step(request: AnalyzeRequest):
    """
    Analyze a STEP file and return features, dimensions, and mesh data.
    """
    try:
        print(f"Analyzing file: {request.fileName}")
        
        # Decode base64 file content
        file_content = base64.b64decode(request.fileBase64)
        
        # Read STEP file
        shape = read_step_file(file_content)
        print("STEP file loaded successfully")
        
        # Get basic properties
        dimensions = get_dimensions(shape)
        volume, surface_area = get_volume_and_area(shape)
        print(f"Dimensions: {dimensions}, Volume: {volume}, Surface Area: {surface_area}")
        
        # Extract features
        features = extract_features(shape)
        print(f"Extracted {len(features)} features")
        
        # Log feature types for debugging
        for f in features:
            print(f"  - {f.type}: {f.parameters}, confidence: {f.metadata.confidence if f.metadata else 'N/A'}, normal: {f.metadata.normalDirection if f.metadata else 'N/A'}")
        
        # Generate mesh with feature mapping
        mesh_data = generate_mesh_data(shape, features)
        print(f"Generated mesh: {len(mesh_data.vertices)//3} vertices, {len(mesh_data.indices)//3} triangles")
        print(f"Triangle feature map length: {len(mesh_data.triangleFeatureMap)}")
        
        # Convert features to dict for response
        features_dict = [f.dict() for f in features]
        
        return AnalyzeResponse(
            success=True,
            features=features_dict,
            dimensions=dimensions,
            volume=volume,
            surfaceArea=surface_area,
            meshData=mesh_data
        )
        
    except Exception as e:
        print(f"Error analyzing STEP file: {str(e)}")
        import traceback
        traceback.print_exc()
        return AnalyzeResponse(
            success=False,
            error=str(e)
        )


@app.post("/analyze-step")
async def analyze_step_alias(request: AnalyzeRequest):
    """Alias endpoint for compatibility."""
    return await analyze_step(request)


if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)
