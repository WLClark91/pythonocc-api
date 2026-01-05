# main.py - Complete pythonOCC API for STEP file analysis with mesh generation
# Updated with proper hole vs boss detection based on surface normal direction
# Now includes FreeCAD integration for SolidWorks file conversion
# 
# VERSION HISTORY:
# v1.0.0 (2025-01-05) - Initial release with basic STEP analysis
# v1.1.0 (2025-01-05) - Added hole vs boss detection using surface normal analysis
# v1.2.0 (2025-01-05) - Added mesh generation with triangle-to-feature mapping
# v1.3.0 (2025-01-05) - Added version endpoint and version tracking
# v1.4.0 (2026-01-05) - Added FreeCAD integration for SolidWorks (SLDPRT/SLDASM) file support
# v1.4.1 (2026-01-05) - Enhanced headless FreeCAD support with xvfb, improved error handling
# v1.5.0 (2026-01-05) - Improved pocket detection with concavity, wall, depth, and boundary analysis
# v1.6.0 (2026-01-05) - Revised pocket criteria: edge-sharing walls (no count), 0.01mm depth, slot classification

API_VERSION = "1.6.0"
API_VERSION_DATE = "2026-01-05"

import base64
import io
import tempfile
import os
import uuid
import subprocess
import shutil
from typing import Optional, Tuple
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# pythonOCC imports
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_REVERSED, TopAbs_WIRE
from OCC.Core.TopExp import topexp
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
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
from OCC.Core.BRepTools import breptools, BRepTools_WireExplorer
from OCC.Core.ShapeAnalysis import ShapeAnalysis_Surface
from OCC.Core.TopoDS import topods

app = FastAPI(title="PythonOCC STEP Analyzer API")

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# =============================================================================
# FREECAD INTEGRATION FOR SOLIDWORKS FILE CONVERSION
# =============================================================================

# Check if FreeCAD is available
FREECAD_AVAILABLE = False
FREECAD_PATH = None
FREECAD_PYTHON_PATH = None

def check_freecad_availability():
    """Check if FreeCAD is installed and available for file conversion."""
    global FREECAD_AVAILABLE, FREECAD_PATH, FREECAD_PYTHON_PATH
    
    # First, look for freecadcmd (headless version - preferred)
    freecadcmd_paths = [
        "/usr/bin/freecadcmd",
        "/usr/local/bin/freecadcmd",
        "/opt/freecad/bin/freecadcmd",
        shutil.which("freecadcmd"),
    ]
    
    for path in freecadcmd_paths:
        if path and os.path.isfile(path) and os.access(path, os.X_OK):
            FREECAD_PATH = path
            FREECAD_AVAILABLE = True
            print(f"FreeCAD (headless) found at: {path}")
            return True
    
    # Fallback to regular freecad
    freecad_paths = [
        "/usr/bin/freecad",
        "/usr/local/bin/freecad",
        "/opt/freecad/bin/freecad",
        shutil.which("freecad"),
    ]
    
    for path in freecad_paths:
        if path and os.path.isfile(path) and os.access(path, os.X_OK):
            FREECAD_PATH = path
            FREECAD_AVAILABLE = True
            print(f"FreeCAD found at: {path}")
            return True
    
    # Check for FreeCAD Python module directly
    try:
        result = subprocess.run(
            ["python3", "-c", "import FreeCAD; print(FreeCAD.__file__)"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            FREECAD_PYTHON_PATH = "python3"
            FREECAD_AVAILABLE = True
            print(f"FreeCAD Python module available via python3")
            return True
    except:
        pass
    
    print("FreeCAD not found - SolidWorks file support will be unavailable")
    return False

# Check FreeCAD on startup
check_freecad_availability()


# FreeCAD conversion script (written to temp file for execution)
# Uses explicit error output to stderr for better capture
FREECAD_CONVERSION_SCRIPT = '''
import sys
import os

# Redirect stdout to capture all output
print(f"FreeCAD conversion starting...", file=sys.stderr)
print(f"Input: {sys.argv[1]}", file=sys.stderr)
print(f"Output: {sys.argv[2]}", file=sys.stderr)

try:
    import FreeCAD
    print(f"FreeCAD version: {FreeCAD.Version()}", file=sys.stderr)
except ImportError as e:
    print(f"IMPORT_ERROR: Cannot import FreeCAD: {e}", file=sys.stderr)
    sys.exit(1)

try:
    import Part
except ImportError as e:
    print(f"IMPORT_ERROR: Cannot import Part module: {e}", file=sys.stderr)
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

try:
    # Verify input file exists
    if not os.path.exists(input_file):
        raise Exception(f"Input file does not exist: {input_file}")
    
    file_size = os.path.getsize(input_file)
    print(f"Input file size: {file_size} bytes", file=sys.stderr)
    
    # Open the document
    print("Opening document...", file=sys.stderr)
    doc = FreeCAD.openDocument(input_file)
    
    if doc is None:
        raise Exception("FreeCAD returned None when opening document - file format may not be supported")
    
    print(f"Document opened: {doc.Name}", file=sys.stderr)
    
    # Get all objects
    objects = doc.Objects
    print(f"Found {len(objects)} objects in document", file=sys.stderr)
    
    if not objects:
        raise Exception("No objects found in document - file may be empty or corrupted")
    
    # Collect all shapes
    shapes = []
    for i, obj in enumerate(objects):
        print(f"Processing object {i}: {obj.Name} (Type: {obj.TypeId})", file=sys.stderr)
        if hasattr(obj, 'Shape') and obj.Shape:
            if not obj.Shape.isNull():
                shapes.append(obj.Shape)
                print(f"  -> Added shape with {len(obj.Shape.Faces)} faces", file=sys.stderr)
            else:
                print(f"  -> Shape is null, skipping", file=sys.stderr)
        else:
            print(f"  -> No shape attribute", file=sys.stderr)
    
    if not shapes:
        raise Exception("No valid shapes found in document - objects exist but have no geometry")
    
    print(f"Collected {len(shapes)} valid shapes", file=sys.stderr)
    
    # Combine shapes into compound if multiple
    if len(shapes) == 1:
        final_shape = shapes[0]
    else:
        print("Creating compound from multiple shapes...", file=sys.stderr)
        final_shape = Part.makeCompound(shapes)
    
    print(f"Final shape has {len(final_shape.Faces)} faces", file=sys.stderr)
    
    # Export to STEP
    print(f"Exporting to STEP: {output_file}", file=sys.stderr)
    final_shape.exportStep(output_file)
    
    # Verify output
    if os.path.exists(output_file):
        out_size = os.path.getsize(output_file)
        print(f"SUCCESS: STEP file created ({out_size} bytes)", file=sys.stderr)
    else:
        raise Exception("Export completed but output file was not created")
    
    # Close document
    FreeCAD.closeDocument(doc.Name)
    print("Document closed, conversion complete", file=sys.stderr)
    
except Exception as e:
    import traceback
    print(f"CONVERSION_ERROR: {str(e)}", file=sys.stderr)
    print(f"Traceback:\\n{traceback.format_exc()}", file=sys.stderr)
    sys.exit(1)
'''


def convert_solidworks_to_step(file_content: bytes, original_filename: str) -> bytes:
    """
    Convert a SolidWorks file (SLDPRT/SLDASM) to STEP format using FreeCAD.
    
    Args:
        file_content: The binary content of the SolidWorks file
        original_filename: Original filename to determine file type
        
    Returns:
        bytes: The converted STEP file content
        
    Raises:
        Exception: If conversion fails or FreeCAD is not available
    """
    if not FREECAD_AVAILABLE:
        raise Exception(
            "FreeCAD is not installed. SolidWorks file conversion is not available. "
            "Please install FreeCAD or convert your file to STEP format manually."
        )
    
    # Determine file extension
    ext = os.path.splitext(original_filename.lower())[1]
    if ext not in ['.sldprt', '.sldasm']:
        raise Exception(f"Unsupported file format: {ext}")
    
    print(f"Starting SolidWorks conversion for: {original_filename} ({len(file_content)} bytes)")
    
    # Create temporary files
    with tempfile.TemporaryDirectory() as tmpdir:
        # Write input file
        input_path = os.path.join(tmpdir, f"input{ext}")
        with open(input_path, 'wb') as f:
            f.write(file_content)
        
        print(f"Wrote input file to: {input_path}")
        
        # Output STEP file
        output_path = os.path.join(tmpdir, "output.step")
        
        # Write conversion script
        script_path = os.path.join(tmpdir, "convert.py")
        with open(script_path, 'w') as f:
            f.write(FREECAD_CONVERSION_SCRIPT)
        
        # Build command based on what's available
        if FREECAD_PATH:
            # Use FreeCAD executable with script
            cmd = [FREECAD_PATH, "-c", script_path, input_path, output_path]
        elif FREECAD_PYTHON_PATH:
            # Use Python with FreeCAD module
            cmd = [FREECAD_PYTHON_PATH, script_path, input_path, output_path]
        else:
            raise Exception("No FreeCAD execution method available")
        
        # Set up environment for headless operation
        env = os.environ.copy()
        env['QT_QPA_PLATFORM'] = 'offscreen'
        env['DISPLAY'] = os.environ.get('DISPLAY', ':99')
        
        print(f"Running FreeCAD conversion: {' '.join(cmd)}")
        print(f"Environment: QT_QPA_PLATFORM={env.get('QT_QPA_PLATFORM')}, DISPLAY={env.get('DISPLAY')}")
        
        # Run FreeCAD conversion
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=180,  # 3 minute timeout for large assemblies
                env=env
            )
            
            # Log all output for debugging
            all_output = []
            if result.stdout:
                print(f"FreeCAD stdout:\n{result.stdout}")
                all_output.append(f"stdout: {result.stdout}")
            if result.stderr:
                print(f"FreeCAD stderr:\n{result.stderr}")
                all_output.append(f"stderr: {result.stderr}")
            
            print(f"FreeCAD exit code: {result.returncode}")
            
            # Check for specific error patterns
            combined_output = (result.stdout or "") + (result.stderr or "")
            
            if "IMPORT_ERROR" in combined_output:
                raise Exception(f"FreeCAD module import failed. Check FreeCAD installation. Details: {combined_output}")
            
            if "CONVERSION_ERROR" in combined_output:
                # Extract the specific error message
                error_lines = [l for l in combined_output.split('\n') if 'CONVERSION_ERROR' in l or 'Traceback' in l]
                raise Exception(f"FreeCAD conversion failed: {' '.join(error_lines) or combined_output}")
            
            if result.returncode != 0:
                error_msg = result.stderr or result.stdout or "Unknown error (no output captured)"
                raise Exception(f"FreeCAD process exited with code {result.returncode}: {error_msg}")
            
            # Read the converted STEP file
            if not os.path.exists(output_path):
                raise Exception(f"FreeCAD completed but output file not found. Output: {combined_output}")
            
            with open(output_path, 'rb') as f:
                step_content = f.read()
            
            if len(step_content) < 100:
                raise Exception(f"FreeCAD produced an empty or invalid STEP file ({len(step_content)} bytes)")
            
            print(f"Successfully converted {original_filename} to STEP ({len(step_content)} bytes)")
            return step_content
            
        except subprocess.TimeoutExpired:
            raise Exception("FreeCAD conversion timed out (>180 seconds). File may be too complex or FreeCAD may be hanging.")
        except Exception as e:
            if "FreeCAD" in str(e):
                raise  # Re-raise our detailed exceptions
            raise Exception(f"FreeCAD conversion error: {str(e)}")


def get_supported_extensions() -> list:
    """Get list of supported file extensions."""
    extensions = [".step", ".stp"]
    if FREECAD_AVAILABLE:
        extensions.extend([".sldprt", ".sldasm"])
    return extensions


def is_solidworks_file(filename: str) -> bool:
    """Check if the file is a SolidWorks format."""
    ext = os.path.splitext(filename.lower())[1]
    return ext in ['.sldprt', '.sldasm']


# =============================================================================
# REQUEST/RESPONSE MODELS
# =============================================================================

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
    convertedFromFormat: Optional[str] = None  # Tracks if file was converted
    error: Optional[str] = None


# =============================================================================
# STEP FILE PROCESSING
# =============================================================================

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


# =============================================================================
# ENHANCED POCKET DETECTION HELPERS
# =============================================================================

def get_shape_center(shape) -> gp_Pnt:
    """Get the center point of the shape's bounding box."""
    bbox = Bnd_Box()
    brepbndlib.Add(shape, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return gp_Pnt((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2)


def check_face_concavity(face, shape) -> Tuple[bool, float]:
    """
    Check if a planar face is concave (material removed) by comparing 
    face normal direction to the vector toward shape center.
    
    A face is considered concave (pocket floor) if its normal points 
    AWAY from the shape's center (indicating an internal cavity).
    
    Returns:
        tuple: (is_concave: bool, confidence: float)
    """
    try:
        adaptor = BRepAdaptor_Surface(face)
        
        # Get face centroid
        face_center = get_face_centroid(face)
        face_pnt = gp_Pnt(face_center.x, face_center.y, face_center.z)
        
        # Get shape center
        shape_center = get_shape_center(shape)
        
        # Vector from face center to shape center
        to_center = gp_Vec(face_pnt, shape_center)
        if to_center.Magnitude() < 1e-10:
            return (False, 0.3)
        to_center.Normalize()
        
        # Get face normal at centroid
        u_min, u_max, v_min, v_max = breptools.UVBounds(face)
        u_mid = (u_min + u_max) / 2.0
        v_mid = (v_min + v_max) / 2.0
        
        props = BRepLProp_SLProps(adaptor, u_mid, v_mid, 1, 1e-6)
        if not props.IsNormalDefined():
            return (False, 0.3)
        
        normal = props.Normal()
        
        # Account for face orientation
        if face.Orientation() == TopAbs_REVERSED:
            normal.Reverse()
        
        normal_vec = gp_Vec(normal.X(), normal.Y(), normal.Z())
        
        # Dot product: positive means normal points toward center (external face)
        # Negative means normal points away from center (concave/pocket floor)
        dot_product = normal_vec.Dot(to_center)
        
        is_concave = dot_product < -0.1  # Threshold to avoid edge cases
        
        # Confidence based on how clearly it points away
        confidence = min(0.95, 0.5 + abs(dot_product) * 0.45) if is_concave else 0.3
        
        return (is_concave, confidence)
        
    except Exception as e:
        print(f"Error checking face concavity: {e}")
        return (False, 0.3)


def build_face_adjacency_map(shape) -> TopTools_IndexedDataMapOfShapeListOfShape:
    """Build a map of edges to their adjacent faces for topology analysis."""
    edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map)
    return edge_face_map


def find_adjacent_enclosing_walls(face, shape, edge_face_map) -> list:
    """
    Find faces that share edges with this face and are roughly vertical
    (potential pocket walls). Includes both planar and curved (e.g., cylindrical) walls.
    
    A wall face has a normal that is roughly perpendicular to the floor normal
    (dot product close to 0). The count of walls does not matter - a circular
    pocket has 1 curved wall, a rectangular pocket has 4 planar walls.
    
    Returns:
        list: List of dicts with adjacent wall face info (edge-sharing walls)
    """
    adjacent_walls = []
    
    try:
        # Get floor face normal
        floor_adaptor = BRepAdaptor_Surface(face)
        u_min, u_max, v_min, v_max = breptools.UVBounds(face)
        u_mid, v_mid = (u_min + u_max) / 2.0, (v_min + v_max) / 2.0
        
        floor_props = BRepLProp_SLProps(floor_adaptor, u_mid, v_mid, 1, 1e-6)
        if not floor_props.IsNormalDefined():
            return adjacent_walls
        
        floor_normal = floor_props.Normal()
        if face.Orientation() == TopAbs_REVERSED:
            floor_normal.Reverse()
        floor_normal_vec = gp_Vec(floor_normal.X(), floor_normal.Y(), floor_normal.Z())
        
        # Explore edges of this face
        edge_explorer = TopExp_Explorer(face, TopAbs_EDGE)
        seen_faces = set()
        wall_index = 0
        
        while edge_explorer.More():
            edge = edge_explorer.Current()
            
            # Find faces sharing this edge
            edge_index = edge_face_map.FindIndex(edge)
            if edge_index > 0:
                adjacent_face_list = edge_face_map.FindFromIndex(edge_index)
                
                for i in range(adjacent_face_list.Size()):
                    adj_face = adjacent_face_list.Value(i + 1)  # 1-indexed
                    
                    # Skip if it's the same face or already processed
                    face_hash = adj_face.__hash__()
                    if adj_face.IsSame(face) or face_hash in seen_faces:
                        continue
                    seen_faces.add(face_hash)
                    
                    # Check adjacent face type (planar or cylindrical walls are valid)
                    adj_adaptor = BRepAdaptor_Surface(topods.Face(adj_face))
                    adj_surface_type = adj_adaptor.GetType()
                    
                    # Accept planar walls and cylindrical walls (for circular pockets)
                    is_planar_wall = adj_surface_type == GeomAbs_Plane
                    is_curved_wall = adj_surface_type == GeomAbs_Cylinder
                    
                    if not (is_planar_wall or is_curved_wall):
                        continue
                    
                    # Get adjacent face normal
                    au_min, au_max, av_min, av_max = breptools.UVBounds(topods.Face(adj_face))
                    au_mid, av_mid = (au_min + au_max) / 2.0, (av_min + av_max) / 2.0
                    
                    adj_props = BRepLProp_SLProps(adj_adaptor, au_mid, av_mid, 1, 1e-6)
                    if not adj_props.IsNormalDefined():
                        continue
                    
                    adj_normal = adj_props.Normal()
                    if adj_face.Orientation() == TopAbs_REVERSED:
                        adj_normal.Reverse()
                    adj_normal_vec = gp_Vec(adj_normal.X(), adj_normal.Y(), adj_normal.Z())
                    
                    # Check if roughly perpendicular to floor (wall)
                    dot = abs(floor_normal_vec.Dot(adj_normal_vec))
                    if dot < 0.3:  # Nearly perpendicular
                        # Get wall face properties
                        wall_props = GProp_GProps()
                        brepgprop.SurfaceProperties(adj_face, wall_props)
                        wall_area = wall_props.Mass()
                        wall_center = wall_props.CentreOfMass()
                        
                        wall_type = "curved" if is_curved_wall else "planar"
                        
                        adjacent_walls.append({
                            "id": f"wall_{wall_index}",
                            "area": wall_area,
                            "center_z": wall_center.Z(),
                            "max_z": get_face_max_z(topods.Face(adj_face)),
                            "wall_type": wall_type
                        })
                        wall_index += 1
            
            edge_explorer.Next()
        
    except Exception as e:
        print(f"Error finding adjacent walls: {e}")
    
    return adjacent_walls


def get_face_max_z(face) -> float:
    """Get the maximum Z coordinate of a face's vertices."""
    max_z = float('-inf')
    
    try:
        vertex_explorer = TopExp_Explorer(face, TopAbs_EDGE)
        while vertex_explorer.More():
            edge = vertex_explorer.Current()
            curve_adaptor = BRepAdaptor_Curve(edge)
            
            # Sample points along edge
            u_min = curve_adaptor.FirstParameter()
            u_max = curve_adaptor.LastParameter()
            
            for t in [0.0, 0.5, 1.0]:
                u = u_min + t * (u_max - u_min)
                pnt = curve_adaptor.Value(u)
                max_z = max(max_z, pnt.Z())
            
            vertex_explorer.Next()
    except:
        pass
    
    return max_z if max_z != float('-inf') else 0.0


def calculate_pocket_depth(floor_face, wall_faces: list) -> float:
    """
    Calculate pocket depth from the highest point of walls to floor centroid Z.
    
    Depth = max(wall_max_z) - floor_z
    """
    try:
        if not wall_faces:
            return 0.0
        
        # Get floor Z level
        floor_center = get_face_centroid(floor_face)
        floor_z = floor_center.z
        
        # Get maximum Z from wall faces
        max_wall_z = max(w.get("max_z", floor_z) for w in wall_faces)
        
        depth = max_wall_z - floor_z
        return max(0.0, depth)
        
    except Exception as e:
        print(f"Error calculating pocket depth: {e}")
        return 0.0


def check_closed_boundary(face) -> bool:
    """
    Check if the face's outer wire forms a closed loop.
    A closed pocket has all edges connected in a continuous loop.
    """
    try:
        outer_wire = breptools.OuterWire(face)
        if outer_wire.IsNull():
            return False
        
        # Check if wire is closed by exploring edges
        wire_explorer = BRepTools_WireExplorer(outer_wire)
        edge_count = 0
        
        while wire_explorer.More():
            edge_count += 1
            wire_explorer.Next()
        
        # A closed wire needs at least 3 edges (triangle) or 4 for rectangle
        return edge_count >= 3
        
    except Exception as e:
        print(f"Error checking boundary: {e}")
        return False


def analyze_planar_face_for_pocket(face, face_id: str, shape, total_surface_area: float, edge_face_map) -> Optional[ExtractedFeature]:
    """
    Enhanced pocket/slot detection with strict qualifying criteria.
    
    QUALIFYING CRITERIA (must ALL pass, or feature is rejected):
    1. Concavity - face normal points away from shape center (material removed)
    2. Enclosing Walls - has wall(s) sharing edges (count doesn't matter: 1 curved or N planar)
    3. Measurable Depth - depth > 0.01mm (epsilon to filter numerical noise)
    4. Closed Boundary - if fails, feature becomes "slot" instead of "pocket"
    
    SUPPORTING CRITERIA (boosts confidence, doesn't disqualify):
    - Relative Area < 15% of total surface
    
    Returns pocket, slot, or None (rejected).
    """
    DEPTH_EPSILON = 0.01  # mm - filters floating-point noise while catching shallow pockets
    
    try:
        props = GProp_GProps()
        brepgprop.SurfaceProperties(face, props)
        area = props.Mass()
        
        # Skip very tiny faces (likely numerical artifacts)
        if area < 1.0:  # Less than 1 mm²
            print(f"  [REJECT] {face_id}: Area too small ({area:.2f} mm²)")
            return None
        
        # =================================================================
        # QUALIFYING CRITERION 1: CONCAVITY (must pass)
        # Face should be concave (material removed, normal points away)
        # =================================================================
        is_concave, concavity_confidence = check_face_concavity(face, shape)
        if not is_concave:
            print(f"  [REJECT] {face_id}: Failed concavity check (external face)")
            return None
        
        # =================================================================
        # QUALIFYING CRITERION 2: ENCLOSING WALLS (must pass)
        # Must have at least 1 wall sharing edges (planar or curved)
        # =================================================================
        adjacent_walls = find_adjacent_enclosing_walls(face, shape, edge_face_map)
        wall_count = len(adjacent_walls)
        has_enclosing_walls = wall_count >= 1
        
        if not has_enclosing_walls:
            print(f"  [REJECT] {face_id}: No enclosing walls found")
            return None
        
        # =================================================================
        # QUALIFYING CRITERION 3: MEASURABLE DEPTH (must pass)
        # Depth must be > epsilon (0.01mm) to filter numerical noise
        # =================================================================
        depth = calculate_pocket_depth(face, adjacent_walls)
        has_meaningful_depth = depth > DEPTH_EPSILON
        
        if not has_meaningful_depth:
            print(f"  [REJECT] {face_id}: Depth too shallow ({depth:.4f}mm < {DEPTH_EPSILON}mm)")
            return None
        
        # =================================================================
        # QUALIFYING CRITERION 4: CLOSED BOUNDARY
        # If fails, classify as "slot" instead of "pocket"
        # =================================================================
        is_closed = check_closed_boundary(face)
        
        # Determine feature type based on boundary closure
        if is_closed:
            feature_type = "pocket"
            pocket_type = "closed"
        else:
            feature_type = "slot"
            pocket_type = "open"
            print(f"  [SLOT] {face_id}: Open boundary - classifying as slot")
        
        # =================================================================
        # SUPPORTING CRITERION: RELATIVE AREA (boosts confidence)
        # Pocket/slot floors are typically < 15% of total surface
        # =================================================================
        relative_area = area / total_surface_area if total_surface_area > 0 else 0
        is_small_relative = relative_area < 0.15
        
        # =================================================================
        # CONFIDENCE CALCULATION
        # =================================================================
        confidence = 0.50  # Base confidence (passed all qualifying criteria)
        confidence += 0.20 * concavity_confidence  # Concavity strength
        confidence += 0.10 if wall_count >= 2 else 0.05  # More walls = higher confidence
        confidence += 0.10 if is_small_relative else 0.0  # Supporting: relative area
        confidence += 0.05 if depth > 1.0 else 0.0  # Deeper = more confident
        
        # Determine wall type for metadata
        wall_types = list(set(w.get("wall_type", "planar") for w in adjacent_walls))
        
        print(f"  [DETECTED] {face_id}: {feature_type} (depth={depth:.2f}mm, walls={wall_count}, closed={is_closed})")
        
        return ExtractedFeature(
            type=feature_type,
            parameters={
                "depth": round(depth, 2),
                "area": round(area, 2),
                "wall_count": wall_count,
                "wall_types": wall_types,
                "is_closed": is_closed,
                "pocket_type": pocket_type
            },
            location=get_face_centroid(face),
            id=str(uuid.uuid4()),
            metadata=FeatureMetadata(
                faceIds=[face_id] + [w["id"] for w in adjacent_walls],
                confidence=round(min(0.95, confidence), 2),
                detectionMethod="strict_qualifying_criteria_v1.6"
            )
        )
        
    except Exception as e:
        print(f"Error in pocket/slot analysis for {face_id}: {e}")
        return None


def analyze_face_for_feature(face, face_id: str) -> Optional[ExtractedFeature]:
    """
    Analyze a single face to detect if it's part of a feature.
    Note: Pocket detection is handled separately in extract_features using enhanced logic.
    """
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
        
        # Planar faces are handled by enhanced pocket detection in extract_features
        # Skip here to avoid duplicate/low-confidence detection
        
        return None
        
    except Exception as e:
        print(f"Error analyzing face {face_id}: {e}")
        return None



def extract_features(shape) -> list:
    """
    Extract all features from the shape using enhanced detection.
    Uses improved pocket detection with concavity, wall, and depth analysis.
    """
    features = []
    
    # Pre-compute total surface area for relative area calculations
    props = GProp_GProps()
    brepgprop.SurfaceProperties(shape, props)
    total_surface_area = props.Mass()
    
    # Build edge-face adjacency map for wall detection
    edge_face_map = build_face_adjacency_map(shape)
    
    # Collect all planar faces for enhanced pocket detection
    planar_faces = []
    
    face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_index = 0
    
    while face_explorer.More():
        face = topods.Face(face_explorer.Current())
        face_id = f"face_{face_index}"
        
        # Check surface type
        adaptor = BRepAdaptor_Surface(face)
        surface_type = adaptor.GetType()
        
        if surface_type == GeomAbs_Plane:
            # Store planar faces for enhanced pocket detection
            planar_faces.append((face, face_id))
        else:
            # Use standard detection for non-planar faces
            feature = analyze_face_for_feature(face, face_id)
            if feature:
                features.append(feature)
        
        face_explorer.Next()
        face_index += 1
    
    # Enhanced pocket detection for planar faces
    for face, face_id in planar_faces:
        pocket_feature = analyze_planar_face_for_pocket(
            face, face_id, shape, total_surface_area, edge_face_map
        )
        if pocket_feature:
            features.append(pocket_feature)
    
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


# =============================================================================
# API ENDPOINTS
# =============================================================================

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy", 
        "service": "pythonocc-api",
        "version": API_VERSION,
        "versionDate": API_VERSION_DATE,
        "freecadAvailable": FREECAD_AVAILABLE,
        "freecadPath": FREECAD_PATH
    }


@app.get("/version")
async def get_version():
    """Get API version information."""
    return {
        "version": API_VERSION,
        "versionDate": API_VERSION_DATE,
        "name": "PythonOCC STEP Analyzer API",
        "supportedFormats": get_supported_extensions(),
        "freecadAvailable": FREECAD_AVAILABLE,
        "changelog": [
            {"version": "1.4.0", "date": "2026-01-05", "changes": [
                "Added FreeCAD integration for SolidWorks file support",
                "SLDPRT and SLDASM files now auto-convert to STEP",
                "Added supported formats endpoint"
            ]},
            {"version": "1.3.0", "date": "2025-01-05", "changes": ["Added version endpoint", "Added version tracking"]},
            {"version": "1.2.0", "date": "2025-01-05", "changes": ["Added mesh generation with triangle-to-feature mapping"]},
            {"version": "1.1.0", "date": "2025-01-05", "changes": ["Added hole vs boss detection using surface normal analysis"]},
            {"version": "1.0.0", "date": "2025-01-05", "changes": ["Initial release with basic STEP analysis"]}
        ]
    }


@app.get("/formats")
async def get_supported_formats():
    """Get list of supported file formats."""
    formats = {
        "native": [".step", ".stp"],
        "convertible": [],
        "all": get_supported_extensions()
    }
    
    if FREECAD_AVAILABLE:
        formats["convertible"] = [".sldprt", ".sldasm"]
    
    return {
        "formats": formats,
        "freecadAvailable": FREECAD_AVAILABLE,
        "note": "Convertible formats require FreeCAD to be installed on the server."
    }


@app.post("/analyze", response_model=AnalyzeResponse)
async def analyze_step(request: AnalyzeRequest):
    """
    Analyze a CAD file and return features, dimensions, and mesh data.
    Supports STEP files natively, and SolidWorks files (SLDPRT/SLDASM) via FreeCAD conversion.
    """
    try:
        print(f"Analyzing file: {request.fileName}")
        
        # Decode base64 file content
        file_content = base64.b64decode(request.fileBase64)
        converted_from = None
        
        # Check if this is a SolidWorks file that needs conversion
        if is_solidworks_file(request.fileName):
            print(f"Detected SolidWorks file, converting to STEP...")
            original_ext = os.path.splitext(request.fileName.lower())[1]
            file_content = convert_solidworks_to_step(file_content, request.fileName)
            converted_from = original_ext.upper().replace(".", "")
            print(f"Conversion complete, proceeding with STEP analysis")
        
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
            meshData=mesh_data,
            convertedFromFormat=converted_from
        )
        
    except Exception as e:
        print(f"Error analyzing CAD file: {str(e)}")
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


@app.post("/convert")
async def convert_file(request: AnalyzeRequest):
    """
    Convert a SolidWorks file to STEP format without full analysis.
    Returns the converted STEP file as base64.
    """
    try:
        if not is_solidworks_file(request.fileName):
            return {
                "success": False,
                "error": "File is not a SolidWorks format. Only SLDPRT and SLDASM files can be converted."
            }
        
        file_content = base64.b64decode(request.fileBase64)
        step_content = convert_solidworks_to_step(file_content, request.fileName)
        
        return {
            "success": True,
            "stepBase64": base64.b64encode(step_content).decode('utf-8'),
            "originalFormat": os.path.splitext(request.fileName.lower())[1].upper().replace(".", ""),
            "originalSize": len(file_content),
            "convertedSize": len(step_content)
        }
        
    except Exception as e:
        return {
            "success": False,
            "error": str(e)
        }


if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)

