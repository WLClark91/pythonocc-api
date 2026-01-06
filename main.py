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
# v1.6.1 (2026-01-05) - Fixed TopExp import: use 'topexp' module (lowercase) for pythonOCC 7.7.0 compatibility
# v1.7.0 (2026-01-05) - Added comprehensive surface detection: planar, cylindrical, conical, spherical, toroidal, freeform
#                       with stock face tagging, area, orientation, and curvature complexity parameters
# v1.7.1 (2026-01-05) - Enhanced hole detection: added arc angle (>=270°) and circular edge criteria to prevent
#                       external curved surfaces from being misclassified as holes
# v1.8.0 (2026-01-06) - Refined cylindrical surface classification:
#                       - hole: inward-facing + circular edge + arc >= 270°
#                       - boss: outward-facing + circular edge + arc >= 270°
#                       - cylindrical_face: outward-facing + has circular edge (any arc)
#                       - curved_surface: all other cylindrical surfaces
# v1.9.0 (2026-01-06) - Enhanced pocket/slot detection with geometric proximity fallback:
#                       - Added edge proximity detection when topological edge sharing fails
#                       - Samples edge endpoints and checks if they lie on adjacent face boundaries
#                       - Resolves pocket detection failures in STEP files with non-shared edges

API_VERSION = "1.9.0"
API_VERSION_DATE = "2026-01-06"

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
    GeomAbs_Cone, GeomAbs_Circle, GeomAbs_Sphere,
    GeomAbs_BezierSurface, GeomAbs_BSplineSurface, GeomAbs_SurfaceOfRevolution,
    GeomAbs_SurfaceOfExtrusion, GeomAbs_OffsetSurface, GeomAbs_OtherSurface
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


def get_cylindrical_face_arc_angle(face, adaptor) -> float:
    """
    Calculate the arc angle (in degrees) covered by a cylindrical face.
    
    A full hole/boss should cover close to 360 degrees.
    Partial cylindrical surfaces (like rounded edges) cover less.
    
    Returns:
        float: Arc angle in degrees (0-360)
    """
    try:
        # Get UV bounds - for a cylinder, U is the angular parameter
        u_min, u_max, v_min, v_max = breptools.UVBounds(face)
        
        # Angular extent in radians
        arc_radians = abs(u_max - u_min)
        arc_degrees = arc_radians * 57.2958  # Convert to degrees
        
        return arc_degrees
        
    except Exception as e:
        print(f"Error calculating arc angle: {e}")
        return 0.0


def check_cylindrical_face_closure(face) -> Tuple[bool, int]:
    """
    Check if a cylindrical face has circular edges (indicating a hole/boss).
    
    True holes/bosses typically have:
    - Two circular edges (through hole or boss with flat ends)
    - One circular edge (blind hole or boss on a surface)
    - The edges should be roughly perpendicular to the cylinder axis
    
    Returns:
        tuple: (has_circular_edges: bool, circular_edge_count: int)
    """
    try:
        from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
        from OCC.Core.GeomAbs import GeomAbs_Circle
        
        circular_edge_count = 0
        
        edge_explorer = TopExp_Explorer(face, TopAbs_EDGE)
        while edge_explorer.More():
            edge = topods.Edge(edge_explorer.Current())
            
            try:
                curve_adaptor = BRepAdaptor_Curve(edge)
                if curve_adaptor.GetType() == GeomAbs_Circle:
                    circular_edge_count += 1
            except:
                pass
            
            edge_explorer.Next()
        
        # Holes/bosses should have at least one circular edge
        has_circular_edges = circular_edge_count >= 1
        
        return (has_circular_edges, circular_edge_count)
        
    except Exception as e:
        print(f"Error checking cylindrical face closure: {e}")
        return (False, 0)


def classify_cylindrical_face(face, adaptor) -> tuple:
    """
    Classify a cylindrical face into one of four categories:
    - hole: inward-facing + circular edge + arc >= 270°
    - boss: outward-facing + circular edge + arc >= 270°
    - cylindrical_face: outward-facing + has any circular edge
    - curved_surface: all other cylindrical surfaces
    
    v1.8.0: Refined classification with clear distinction between feature types.
    
    Returns:
        tuple: (classification: str, confidence: float, normal_direction: str)
               classification is one of: 'hole', 'boss', 'cylindrical_face', 'curved_surface'
    """
    try:
        cylinder = adaptor.Cylinder()
        radius = cylinder.Radius()
        axis = cylinder.Axis()
        axis_dir = axis.Direction()
        axis_loc = axis.Location()
        
        # =================================================================
        # CRITERION 1: Arc angle (how much of a circle it covers)
        # =================================================================
        arc_angle = get_cylindrical_face_arc_angle(face, adaptor)
        is_mostly_closed = arc_angle >= 270  # At least 270° of arc
        
        # =================================================================
        # CRITERION 2: Circular edge check (indicates proper cylindrical feature)
        # =================================================================
        has_circular_edges, circular_edge_count = check_cylindrical_face_closure(face)
        
        # =================================================================
        # CRITERION 3: Surface normal direction (determines inward vs outward)
        # =================================================================
        u_min, u_max, v_min, v_max = breptools.UVBounds(face)
        u_mid = (u_min + u_max) / 2.0
        v_mid = (v_min + v_max) / 2.0
        
        surface_pnt = adaptor.Value(u_mid, v_mid)
        props = BRepLProp_SLProps(adaptor, u_mid, v_mid, 1, 1e-6)
        
        if not props.IsNormalDefined():
            return ("curved_surface", 0.5, "undefined")
        
        normal = props.Normal()
        
        # Account for face orientation (REVERSED means normal is flipped)
        if face.Orientation() == TopAbs_REVERSED:
            normal.Reverse()
        
        # Calculate radial direction: from axis to surface point
        point_vec = gp_Vec(axis_loc, surface_pnt)
        axis_vec = gp_Vec(axis_dir)
        proj_length = point_vec.Dot(axis_vec)
        
        axis_point = gp_Pnt(
            axis_loc.X() + axis_dir.X() * proj_length,
            axis_loc.Y() + axis_dir.Y() * proj_length,
            axis_loc.Z() + axis_dir.Z() * proj_length
        )
        
        radial_vec = gp_Vec(axis_point, surface_pnt)
        if radial_vec.Magnitude() > 1e-10:
            radial_vec.Normalize()
        else:
            return ("curved_surface", 0.5, "on_axis")
        
        normal_vec = gp_Vec(normal.X(), normal.Y(), normal.Z())
        dot_product = normal_vec.Dot(radial_vec)
        
        is_outward = dot_product > 0
        normal_direction = "outward" if is_outward else "inward"
        
        # =================================================================
        # CLASSIFICATION LOGIC (v1.8.0)
        # =================================================================
        
        # HOLE: inward-facing + circular edge + arc >= 270°
        if not is_outward and has_circular_edges and is_mostly_closed:
            confidence = 0.70
            confidence += 0.10 if arc_angle >= 350 else 0.05
            confidence += 0.10 if circular_edge_count >= 2 else 0.05
            confidence += 0.05 * min(1.0, abs(dot_product))
            print(f"  [HOLE] arc={arc_angle:.1f}°, circular_edges={circular_edge_count}, r={radius:.2f}mm")
            return ("hole", min(0.95, confidence), normal_direction)
        
        # BOSS: outward-facing + circular edge + arc >= 270°
        if is_outward and has_circular_edges and is_mostly_closed:
            confidence = 0.70
            confidence += 0.10 if arc_angle >= 350 else 0.05
            confidence += 0.10 if circular_edge_count >= 2 else 0.05
            confidence += 0.05 * min(1.0, abs(dot_product))
            print(f"  [BOSS] arc={arc_angle:.1f}°, circular_edges={circular_edge_count}, r={radius:.2f}mm")
            return ("boss", min(0.95, confidence), normal_direction)
        
        # CYLINDRICAL_FACE: outward-facing + has circular edge (any arc)
        if is_outward and has_circular_edges:
            confidence = 0.75
            confidence += 0.10 if arc_angle >= 180 else 0.0
            print(f"  [CYLINDRICAL_FACE] arc={arc_angle:.1f}°, circular_edges={circular_edge_count}, r={radius:.2f}mm")
            return ("cylindrical_face", min(0.90, confidence), normal_direction)
        
        # CURVED_SURFACE: all other cylindrical surfaces
        print(f"  [CURVED_SURFACE] arc={arc_angle:.1f}°, circular_edges={circular_edge_count}, r={radius:.2f}mm, dir={normal_direction}")
        return ("curved_surface", 0.70, normal_direction)
        
    except Exception as e:
        print(f"Error classifying cylindrical face: {e}")
        return ("curved_surface", 0.5, "error")


# =============================================================================
# SURFACE DETECTION HELPERS (v1.7.0)
# =============================================================================

def get_shape_center(shape) -> gp_Pnt:
    """Get the center point of the shape's bounding box."""
    bbox = Bnd_Box()
    brepbndlib.Add(shape, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return gp_Pnt((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2)


def get_shape_bounding_box_expanded(shape, tolerance: float = 0.5) -> dict:
    """Get shape bounding box with tolerance for stock face detection."""
    bbox = Bnd_Box()
    brepbndlib.Add(shape, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return {
        "xmin": xmin - tolerance,
        "xmax": xmax + tolerance,
        "ymin": ymin - tolerance,
        "ymax": ymax + tolerance,
        "zmin": zmin - tolerance,
        "zmax": zmax + tolerance
    }


def is_stock_face(face, bbox_expanded: dict) -> Tuple[bool, Optional[str]]:
    """
    Determine if a face is at the bounding box boundary (stock material face).
    
    Stock faces are faces whose centroids lie on or very near the bounding box
    boundary - these are typically the outer surfaces of raw stock material.
    
    Returns:
        tuple: (is_stock: bool, boundary_type: Optional[str])
               boundary_type is one of: 'top', 'bottom', 'front', 'back', 'left', 'right'
    """
    try:
        centroid = get_face_centroid(face)
        x, y, z = centroid.x, centroid.y, centroid.z
        
        tolerance = 0.5  # mm tolerance for boundary detection
        
        # Check each boundary
        if abs(z - bbox_expanded["zmax"]) < tolerance:
            return (True, "top")
        if abs(z - bbox_expanded["zmin"]) < tolerance:
            return (True, "bottom")
        if abs(y - bbox_expanded["ymax"]) < tolerance:
            return (True, "back")
        if abs(y - bbox_expanded["ymin"]) < tolerance:
            return (True, "front")
        if abs(x - bbox_expanded["xmax"]) < tolerance:
            return (True, "right")
        if abs(x - bbox_expanded["xmin"]) < tolerance:
            return (True, "left")
        
        return (False, None)
        
    except Exception as e:
        print(f"Error checking stock face: {e}")
        return (False, None)


def get_surface_orientation(face, adaptor) -> dict:
    """
    Get the surface normal/orientation at the face centroid.
    
    Returns:
        dict with 'normal' (x, y, z), 'angle_from_z' (degrees from vertical),
        and 'orientation' (human-readable: 'horizontal', 'vertical', 'angled')
    """
    try:
        u_min, u_max, v_min, v_max = breptools.UVBounds(face)
        u_mid = (u_min + u_max) / 2.0
        v_mid = (v_min + v_max) / 2.0
        
        props = BRepLProp_SLProps(adaptor, u_mid, v_mid, 1, 1e-6)
        
        if not props.IsNormalDefined():
            return {"normal": None, "angle_from_z": None, "orientation": "unknown"}
        
        normal = props.Normal()
        
        # Account for face orientation
        if face.Orientation() == TopAbs_REVERSED:
            normal.Reverse()
        
        nx, ny, nz = normal.X(), normal.Y(), normal.Z()
        
        # Calculate angle from Z axis (vertical)
        import math
        z_axis = gp_Vec(0, 0, 1)
        normal_vec = gp_Vec(nx, ny, nz)
        angle_rad = z_axis.Angle(normal_vec)
        angle_deg = math.degrees(angle_rad)
        
        # Classify orientation
        if angle_deg < 15 or angle_deg > 165:
            orientation = "horizontal"
        elif 75 < angle_deg < 105:
            orientation = "vertical"
        else:
            orientation = "angled"
        
        return {
            "normal": {"x": round(nx, 4), "y": round(ny, 4), "z": round(nz, 4)},
            "angle_from_z": round(angle_deg, 1),
            "orientation": orientation
        }
        
    except Exception as e:
        print(f"Error getting surface orientation: {e}")
        return {"normal": None, "angle_from_z": None, "orientation": "unknown"}


def get_curvature_complexity(adaptor, surface_type) -> dict:
    """
    Determine the curvature complexity of a surface for machining classification.
    
    Returns:
        dict with 'complexity' ('simple', 'moderate', 'complex'),
        'requires_axis' (3, 4, or 5 axis machining), and 'curvature_type'
    """
    complexity_map = {
        GeomAbs_Plane: ("simple", 3, "flat"),
        GeomAbs_Cylinder: ("simple", 3, "single_curved"),
        GeomAbs_Cone: ("simple", 4, "single_curved_variable"),
        GeomAbs_Sphere: ("moderate", 4, "double_curved_uniform"),
        GeomAbs_Torus: ("moderate", 4, "double_curved"),
        GeomAbs_BezierSurface: ("complex", 5, "freeform"),
        GeomAbs_BSplineSurface: ("complex", 5, "freeform"),
        GeomAbs_SurfaceOfRevolution: ("moderate", 4, "revolution"),
        GeomAbs_SurfaceOfExtrusion: ("moderate", 3, "extrusion"),
        GeomAbs_OffsetSurface: ("complex", 5, "offset"),
        GeomAbs_OtherSurface: ("complex", 5, "other"),
    }
    
    result = complexity_map.get(surface_type, ("complex", 5, "unknown"))
    
    return {
        "complexity": result[0],
        "requires_axis": result[1],
        "curvature_type": result[2]
    }


def surface_type_to_feature_type(surface_type, cylindrical_classification: str = None) -> str:
    """
    Map OCC surface type to feature type string.
    
    For cylindrical surfaces, use the cylindrical_classification parameter
    to distinguish between cylindrical_face and curved_surface.
    """
    # For cylindrical surfaces, use the classification result
    if surface_type == GeomAbs_Cylinder and cylindrical_classification:
        return cylindrical_classification
    
    type_map = {
        GeomAbs_Plane: "planar_surface",
        GeomAbs_Cylinder: "curved_surface",  # Default fallback for cylinder
        GeomAbs_Cone: "conical_surface",
        GeomAbs_Sphere: "spherical_surface",
        GeomAbs_Torus: "toroidal_surface",
        GeomAbs_BezierSurface: "freeform_surface",
        GeomAbs_BSplineSurface: "freeform_surface",
        GeomAbs_SurfaceOfRevolution: "freeform_surface",
        GeomAbs_SurfaceOfExtrusion: "freeform_surface",
        GeomAbs_OffsetSurface: "freeform_surface",
        GeomAbs_OtherSurface: "freeform_surface",
    }
    return type_map.get(surface_type, "freeform_surface")


def analyze_face_as_surface(face, face_id: str, adaptor, surface_type, bbox_expanded: dict) -> ExtractedFeature:
    """
    Analyze any face as a surface feature with full parameters.
    
    v1.8.0: For cylindrical surfaces, uses classify_cylindrical_face to determine
    whether it should be cylindrical_face or curved_surface.
    
    This captures ALL machined surfaces including:
    - Planar (flat) surfaces
    - Cylindrical faces (outward + circular edge)
    - Curved surfaces (other cylindrical)
    - Conical surfaces
    - Spherical surfaces
    - Toroidal surfaces
    - Freeform/NURBS surfaces
    """
    try:
        # Get surface area
        props = GProp_GProps()
        brepgprop.SurfaceProperties(face, props)
        area = props.Mass()
        
        # Get centroid
        centroid = get_face_centroid(face)
        
        # Get orientation/normal
        orientation = get_surface_orientation(face, adaptor)
        
        # Get curvature complexity
        curvature = get_curvature_complexity(adaptor, surface_type)
        
        # Check if stock face
        is_stock, stock_boundary = is_stock_face(face, bbox_expanded)
        
        # Determine feature type - special handling for cylindrical surfaces
        cylindrical_classification = None
        if surface_type == GeomAbs_Cylinder:
            classification, conf, normal_dir = classify_cylindrical_face(face, adaptor)
            # Only use classification if it's a surface type (not hole/boss)
            if classification in ("cylindrical_face", "curved_surface"):
                cylindrical_classification = classification
        
        feature_type = surface_type_to_feature_type(surface_type, cylindrical_classification)
        
        # Surface-specific parameters
        params = {
            "area": round(area, 2),
            "orientation": orientation["orientation"],
            "angle_from_vertical": orientation["angle_from_z"],
            "normal": orientation["normal"],
            "curvature_complexity": curvature["complexity"],
            "curvature_type": curvature["curvature_type"],
            "requires_axis_count": curvature["requires_axis"],
            "isStockFace": is_stock,
        }
        
        if is_stock:
            params["stockBoundary"] = stock_boundary
        
        # Add geometry-specific parameters
        if surface_type == GeomAbs_Cylinder:
            cylinder = adaptor.Cylinder()
            params["radius"] = round(cylinder.Radius(), 2)
        elif surface_type == GeomAbs_Cone:
            cone = adaptor.Cone()
            params["semi_angle"] = round(abs(cone.SemiAngle()) * 57.2958, 1)  # degrees
            params["ref_radius"] = round(cone.RefRadius(), 2)
        elif surface_type == GeomAbs_Sphere:
            sphere = adaptor.Sphere()
            params["radius"] = round(sphere.Radius(), 2)
        elif surface_type == GeomAbs_Torus:
            torus = adaptor.Torus()
            params["major_radius"] = round(torus.MajorRadius(), 2)
            params["minor_radius"] = round(torus.MinorRadius(), 2)
        
        # Confidence based on surface simplicity
        confidence_map = {"simple": 0.95, "moderate": 0.85, "complex": 0.75}
        confidence = confidence_map.get(curvature["complexity"], 0.75)
        
        return ExtractedFeature(
            type=feature_type,
            parameters=params,
            location=centroid,
            id=str(uuid.uuid4()),
            metadata=FeatureMetadata(
                faceIds=[face_id],
                confidence=confidence,
                detectionMethod="surface_analysis_v1.8.0"
            )
        )
        
    except Exception as e:
        print(f"Error analyzing face as surface {face_id}: {e}")
        return None


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


def get_edge_sample_points(edge, num_samples: int = 5) -> list:
    """
    Get sample points along an edge for geometric proximity checking.
    Returns list of (x, y, z) tuples.
    """
    try:
        curve_adaptor = BRepAdaptor_Curve(edge)
        u_min = curve_adaptor.FirstParameter()
        u_max = curve_adaptor.LastParameter()
        
        points = []
        for i in range(num_samples):
            t = i / (num_samples - 1) if num_samples > 1 else 0.5
            u = u_min + t * (u_max - u_min)
            pnt = curve_adaptor.Value(u)
            points.append((pnt.X(), pnt.Y(), pnt.Z()))
        
        return points
    except:
        return []


def point_near_face_boundary(point: tuple, face, tolerance: float = 0.1) -> bool:
    """
    Check if a point lies near the boundary of a face.
    Uses distance to face edges as the metric.
    """
    try:
        px, py, pz = point
        test_point = gp_Pnt(px, py, pz)
        
        # Explore edges of the face and check distance
        edge_explorer = TopExp_Explorer(face, TopAbs_EDGE)
        while edge_explorer.More():
            edge = edge_explorer.Current()
            curve_adaptor = BRepAdaptor_Curve(edge)
            
            # Sample points on this edge and find minimum distance
            u_min = curve_adaptor.FirstParameter()
            u_max = curve_adaptor.LastParameter()
            
            for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
                u = u_min + t * (u_max - u_min)
                edge_pnt = curve_adaptor.Value(u)
                dist = test_point.Distance(edge_pnt)
                
                if dist < tolerance:
                    return True
            
            edge_explorer.Next()
        
        return False
    except:
        return False


def find_geometrically_adjacent_walls(floor_face, floor_normal_vec, shape, seen_faces: set, tolerance: float = 0.5) -> list:
    """
    Find wall faces that are geometrically adjacent to the floor face.
    This is a fallback when topological edge sharing detection fails.
    
    Checks if floor edge sample points lie near the boundaries of potential wall faces.
    """
    adjacent_walls = []
    wall_index = 0
    
    try:
        # Collect sample points from all edges of the floor face
        floor_edge_points = []
        edge_explorer = TopExp_Explorer(floor_face, TopAbs_EDGE)
        while edge_explorer.More():
            edge = edge_explorer.Current()
            floor_edge_points.extend(get_edge_sample_points(edge, 3))
            edge_explorer.Next()
        
        if not floor_edge_points:
            return adjacent_walls
        
        # Check all faces in the shape
        face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while face_explorer.More():
            candidate_face = topods.Face(face_explorer.Current())
            face_hash = candidate_face.__hash__()
            
            # Skip if same face or already seen
            if candidate_face.IsSame(floor_face) or face_hash in seen_faces:
                face_explorer.Next()
                continue
            
            # Check surface type (only planar and cylindrical walls)
            adaptor = BRepAdaptor_Surface(candidate_face)
            surface_type = adaptor.GetType()
            is_planar_wall = surface_type == GeomAbs_Plane
            is_curved_wall = surface_type == GeomAbs_Cylinder
            
            if not (is_planar_wall or is_curved_wall):
                face_explorer.Next()
                continue
            
            # Get candidate face normal
            u_min, u_max, v_min, v_max = breptools.UVBounds(candidate_face)
            u_mid, v_mid = (u_min + u_max) / 2.0, (v_min + v_max) / 2.0
            
            props = BRepLProp_SLProps(adaptor, u_mid, v_mid, 1, 1e-6)
            if not props.IsNormalDefined():
                face_explorer.Next()
                continue
            
            normal = props.Normal()
            if candidate_face.Orientation() == TopAbs_REVERSED:
                normal.Reverse()
            candidate_normal_vec = gp_Vec(normal.X(), normal.Y(), normal.Z())
            
            # Check if roughly perpendicular to floor (wall criterion)
            dot = abs(floor_normal_vec.Dot(candidate_normal_vec))
            if dot >= 0.3:  # Not perpendicular enough
                face_explorer.Next()
                continue
            
            # Check geometric proximity: do any floor edge points lie near this face's boundary?
            points_near_boundary = 0
            for point in floor_edge_points:
                if point_near_face_boundary(point, candidate_face, tolerance):
                    points_near_boundary += 1
            
            # Require at least 2 points to be near boundary for confidence
            if points_near_boundary >= 2:
                seen_faces.add(face_hash)
                
                wall_props = GProp_GProps()
                brepgprop.SurfaceProperties(candidate_face, wall_props)
                wall_area = wall_props.Mass()
                wall_center = wall_props.CentreOfMass()
                
                wall_type = "curved" if is_curved_wall else "planar"
                
                adjacent_walls.append({
                    "id": f"geom_wall_{wall_index}",
                    "area": wall_area,
                    "center_z": wall_center.Z(),
                    "max_z": get_face_max_z(candidate_face),
                    "wall_type": wall_type,
                    "detection": "geometric_proximity"
                })
                wall_index += 1
                print(f"    [GEOM] Found wall via geometric proximity ({points_near_boundary} points matched)")
            
            face_explorer.Next()
        
    except Exception as e:
        print(f"Error in geometric wall detection: {e}")
    
    return adjacent_walls


def find_adjacent_enclosing_walls(face, shape, edge_face_map) -> list:
    """
    Find faces that share edges with this face and are roughly vertical
    (potential pocket walls). Includes both planar and curved (e.g., cylindrical) walls.
    
    v1.9.0: Now includes geometric proximity fallback when topological edge sharing fails.
    
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
        edges_with_no_match = 0
        total_edges = 0
        
        while edge_explorer.More():
            edge = edge_explorer.Current()
            total_edges += 1
            
            # Find faces sharing this edge (topological approach)
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
                            "wall_type": wall_type,
                            "detection": "topological"
                        })
                        wall_index += 1
            else:
                edges_with_no_match += 1
            
            edge_explorer.Next()
        
        # v1.9.0: GEOMETRIC PROXIMITY FALLBACK
        # If topological edge detection found no walls, try geometric proximity
        if len(adjacent_walls) == 0 and total_edges > 0:
            print(f"    [FALLBACK] Topological detection found 0 walls ({edges_with_no_match}/{total_edges} edges unmatched). Trying geometric proximity...")
            geometric_walls = find_geometrically_adjacent_walls(face, floor_normal_vec, shape, seen_faces, tolerance=0.5)
            adjacent_walls.extend(geometric_walls)
            
            if len(geometric_walls) > 0:
                print(f"    [FALLBACK] Geometric proximity found {len(geometric_walls)} walls")
        
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
    Analyze a single face to detect if it's part of a manufacturing feature.
    
    v1.8.0: Uses classify_cylindrical_face for refined cylindrical classification.
    Returns manufacturing features (hole, boss) only. Surface features
    (cylindrical_face, curved_surface) are handled in the surface detection pass.
    
    Note: Pocket detection is handled separately in extract_features using enhanced logic.
    """
    try:
        adaptor = BRepAdaptor_Surface(face)
        surface_type = adaptor.GetType()
        
        # Cylindrical face -> could be hole, boss, cylindrical_face, or curved_surface
        if surface_type == GeomAbs_Cylinder:
            cylinder = adaptor.Cylinder()
            radius = cylinder.Radius()
            
            # Estimate depth/height from face bounds
            umin, umax, vmin, vmax = adaptor.FirstUParameter(), adaptor.LastUParameter(), \
                                      adaptor.FirstVParameter(), adaptor.LastVParameter()
            height_or_depth = abs(vmax - vmin)
            
            # Use v1.8.0 classification
            classification, confidence, normal_direction = classify_cylindrical_face(face, adaptor)
            
            # Only return manufacturing features (hole, boss)
            # cylindrical_face and curved_surface are handled in surface detection
            if classification not in ("hole", "boss"):
                return None
            
            if classification == "boss":
                params = {"diameter": round(radius * 2, 2), "height": round(height_or_depth, 2)}
            else:
                params = {"diameter": round(radius * 2, 2), "depth": round(height_or_depth, 2)}
            
            return ExtractedFeature(
                type=classification,
                parameters=params,
                location=get_face_centroid(face),
                id=str(uuid.uuid4()),
                metadata=FeatureMetadata(
                    faceIds=[face_id],
                    confidence=confidence,
                    detectionMethod="cylindrical_classification_v1.8.0",
                    normalDirection=normal_direction
                )
            )
        
        # Toroidal face -> fillet
        elif surface_type == GeomAbs_Torus:
            torus = adaptor.Torus()
            minor_radius = torus.MinorRadius()
            
            return ExtractedFeature(
                type="fillet",
                parameters={"radius": round(minor_radius, 2)},
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
                parameters={"angle": round(abs(angle) * 57.2958, 1), "width": 2.0},  # Convert to degrees
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



def extract_features(shape, include_surfaces: bool = True) -> list:
    """
    Extract all features from the shape using enhanced detection.
    
    v1.7.0: Now detects ALL surfaces as features, with stock face tagging.
    
    Feature types detected:
    - Manufacturing features: hole, boss, pocket, slot, fillet, chamfer, thread
    - Surface features: planar_surface, cylindrical_surface, conical_surface,
                        spherical_surface, toroidal_surface, freeform_surface
    
    Args:
        shape: The TopoDS_Shape to analyze
        include_surfaces: If True, detect all surfaces as features (default: True)
        
    Returns:
        list of ExtractedFeature
    """
    manufacturing_features = []
    surface_features = []
    
    # Pre-compute total surface area for relative area calculations
    props = GProp_GProps()
    brepgprop.SurfaceProperties(shape, props)
    total_surface_area = props.Mass()
    
    # Build edge-face adjacency map for wall detection
    edge_face_map = build_face_adjacency_map(shape)
    
    # Get bounding box for stock face detection
    bbox_expanded = get_shape_bounding_box_expanded(shape)
    
    # Collect faces for various detection passes
    planar_faces = []
    all_faces = []
    
    face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_index = 0
    
    while face_explorer.More():
        face = topods.Face(face_explorer.Current())
        face_id = f"face_{face_index}"
        
        # Check surface type
        adaptor = BRepAdaptor_Surface(face)
        surface_type = adaptor.GetType()
        
        # Store all faces for surface detection
        all_faces.append((face, face_id, adaptor, surface_type))
        
        if surface_type == GeomAbs_Plane:
            # Store planar faces for enhanced pocket detection
            planar_faces.append((face, face_id))
        else:
            # Use standard detection for non-planar faces (holes, bosses, fillets, chamfers)
            feature = analyze_face_for_feature(face, face_id)
            if feature:
                manufacturing_features.append(feature)
        
        face_explorer.Next()
        face_index += 1
    
    # Enhanced pocket detection for planar faces
    detected_pocket_face_ids = set()
    for face, face_id in planar_faces:
        pocket_feature = analyze_planar_face_for_pocket(
            face, face_id, shape, total_surface_area, edge_face_map
        )
        if pocket_feature:
            manufacturing_features.append(pocket_feature)
            detected_pocket_face_ids.add(face_id)
    
    # Collect face IDs already detected as manufacturing features
    detected_manufacturing_face_ids = set()
    for feat in manufacturing_features:
        if feat.metadata and feat.metadata.faceIds:
            detected_manufacturing_face_ids.update(feat.metadata.faceIds)
    detected_manufacturing_face_ids.update(detected_pocket_face_ids)
    
    # Surface detection for ALL faces (v1.7.0)
    if include_surfaces:
        print(f"[SURFACE DETECTION] Analyzing {len(all_faces)} faces for surface features")
        
        for face, face_id, adaptor, surface_type in all_faces:
            # Skip faces already detected as manufacturing features
            if face_id in detected_manufacturing_face_ids:
                continue
            
            # Detect as surface feature
            surface_feature = analyze_face_as_surface(
                face, face_id, adaptor, surface_type, bbox_expanded
            )
            if surface_feature:
                surface_features.append(surface_feature)
        
        print(f"[SURFACE DETECTION] Found {len(surface_features)} surface features")
    
    # Combine all features
    all_features = manufacturing_features + surface_features
    
    # Consolidate similar manufacturing features (e.g., multiple hole faces -> one hole with count)
    # Don't consolidate surfaces - each surface should be reported individually
    consolidated_mfg = consolidate_features(manufacturing_features)
    
    # Return manufacturing features (consolidated) + surface features (individual)
    return consolidated_mfg + surface_features


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
            {"version": "1.7.0", "date": "2026-01-05", "changes": [
                "Added comprehensive surface detection for ALL faces",
                "New surface types: planar_surface, cylindrical_surface, conical_surface, spherical_surface, toroidal_surface, freeform_surface",
                "Stock face detection with boundary tagging (top, bottom, front, back, left, right)",
                "Surface parameters: area, orientation, normal, curvature complexity, axis requirements"
            ]},
            {"version": "1.6.1", "date": "2026-01-05", "changes": ["Fixed TopExp import for pythonOCC 7.7.0 compatibility"]},
            {"version": "1.6.0", "date": "2026-01-05", "changes": ["Revised pocket criteria with slot classification"]},
            {"version": "1.5.0", "date": "2026-01-05", "changes": ["Improved pocket detection with concavity, wall, depth analysis"]},
            {"version": "1.4.1", "date": "2026-01-05", "changes": ["Enhanced headless FreeCAD support with xvfb"]},
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
