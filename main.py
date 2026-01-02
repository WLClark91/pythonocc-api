import logging
import base64
import tempfile
import os

# Configure logging first
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Import OCC at module level (loaded at startup, not per-request)
# This prevents timeout/memory issues during request handling
try:
    from OCC.Core.STEPControl import STEPControl_Reader
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.BRepGProp import brepgprop_VolumeProperties, brepgprop_SurfaceProperties
    from OCC.Core.GProp import GProp_GProps
    from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_SOLID
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib_Add
    from OCC.Core.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
    from OCC.Core.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Cone, GeomAbs_Sphere, GeomAbs_Torus, GeomAbs_Circle
    logger.info("OCC libraries loaded successfully at startup")
except ImportError as e:
    logger.fatal(f"FATAL: Failed to import OCC libraries: {e}")
    raise

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any

app = FastAPI(title="PythonOCC Feature Extraction API")

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class AnalyzeRequest(BaseModel):
    fileBase64: str
    fileName: str


class FeatureLocation(BaseModel):
    x: float
    y: float
    z: float


class ExtractedFeature(BaseModel):
    type: str
    parameters: Dict[str, float]
    location: Optional[FeatureLocation] = None
    count: Optional[int] = None


class AnalyzeResponse(BaseModel):
    success: bool
    features: Optional[List[ExtractedFeature]] = None
    dimensions: Optional[Dict[str, float]] = None
    volume: Optional[float] = None
    surfaceArea: Optional[float] = None
    error: Optional[str] = None


@app.get("/")
async def root():
    return {"status": "ok", "service": "pythonOCC Feature Extraction API"}


@app.get("/health")
async def health():
    return {"status": "ok", "occ_loaded": True}


def analyze_shape(shape):
    """Analyze a shape and extract features."""
    features = []
    
    # Count different surface types
    planar_faces = 0
    cylindrical_faces = 0
    conical_faces = 0
    spherical_faces = 0
    toroidal_faces = 0
    other_faces = 0
    
    # Analyze faces
    face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while face_explorer.More():
        face = face_explorer.Current()
        try:
            surface = BRepAdaptor_Surface(face)
            surface_type = surface.GetType()
            
            if surface_type == GeomAbs_Plane:
                planar_faces += 1
            elif surface_type == GeomAbs_Cylinder:
                cylindrical_faces += 1
            elif surface_type == GeomAbs_Cone:
                conical_faces += 1
            elif surface_type == GeomAbs_Sphere:
                spherical_faces += 1
            elif surface_type == GeomAbs_Torus:
                toroidal_faces += 1
            else:
                other_faces += 1
        except Exception as e:
            logger.warning(f"Error analyzing face: {e}")
            other_faces += 1
        
        face_explorer.Next()
    
    # Detect holes (cylindrical faces often indicate holes)
    if cylindrical_faces > 0:
        features.append(ExtractedFeature(
            type="hole",
            parameters={"diameter": 10.0, "depth": 15.0},  # Placeholder values
            count=cylindrical_faces
        ))
    
    # Detect fillets (toroidal faces often indicate fillets)
    if toroidal_faces > 0:
        features.append(ExtractedFeature(
            type="fillet",
            parameters={"radius": 2.0},
            count=toroidal_faces
        ))
    
    # Detect pockets (based on face topology - simplified)
    if planar_faces > 6:  # More than a simple box
        pocket_count = max(1, (planar_faces - 6) // 4)
        features.append(ExtractedFeature(
            type="pocket",
            parameters={"length": 50.0, "width": 30.0, "depth": 10.0},
            count=pocket_count
        ))
    
    # Detect chamfers (conical faces often indicate chamfers)
    if conical_faces > 0:
        features.append(ExtractedFeature(
            type="chamfer",
            parameters={"distance": 1.5, "angle": 45.0},
            count=conical_faces
        ))
    
    # Count edges for potential thread detection
    edge_count = 0
    edge_explorer = TopExp_Explorer(shape, TopAbs_EDGE)
    while edge_explorer.More():
        edge_count += 1
        edge_explorer.Next()
    
    # High edge count with cylindrical faces might indicate threads
    if cylindrical_faces > 0 and edge_count > 50:
        features.append(ExtractedFeature(
            type="thread",
            parameters={"diameter": 8.0, "pitch": 1.25},
            count=1
        ))
    
    logger.info(f"Face analysis: planar={planar_faces}, cylindrical={cylindrical_faces}, "
                f"conical={conical_faces}, spherical={spherical_faces}, "
                f"toroidal={toroidal_faces}, other={other_faces}")
    
    return features


@app.post("/analyze", response_model=AnalyzeResponse)
async def analyze(request: AnalyzeRequest):
    logger.info(f"Received analyze request for: {request.fileName}")
    
    temp_file = None
    try:
        # Decode base64 file content
        try:
            file_content = base64.b64decode(request.fileBase64)
        except Exception as e:
            logger.error(f"Failed to decode base64: {e}")
            return AnalyzeResponse(success=False, error=f"Invalid base64 encoding: {str(e)}")
        
        # Write to temporary file
        suffix = os.path.splitext(request.fileName)[1] or '.step'
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=suffix)
        temp_file.write(file_content)
        temp_file.close()
        
        logger.info(f"Wrote {len(file_content)} bytes to temp file: {temp_file.name}")
        
        # Read STEP file
        step_reader = STEPControl_Reader()
        status = step_reader.ReadFile(temp_file.name)
        
        if status != IFSelect_RetDone:
            logger.error(f"Failed to read STEP file, status: {status}")
            return AnalyzeResponse(success=False, error=f"Failed to read STEP file (status: {status})")
        
        # Transfer to shape
        step_reader.TransferRoots()
        shape = step_reader.OneShape()
        
        if shape.IsNull():
            logger.error("Shape is null after transfer")
            return AnalyzeResponse(success=False, error="Could not extract shape from STEP file")
        
        logger.info("Successfully loaded STEP file, analyzing geometry...")
        
        # Calculate bounding box
        bbox = Bnd_Box()
        brepbndlib_Add(shape, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        
        dimensions = {
            "length": round(abs(xmax - xmin), 2),
            "width": round(abs(ymax - ymin), 2),
            "height": round(abs(zmax - zmin), 2)
        }
        
        logger.info(f"Bounding box: {dimensions}")
        
        # Calculate volume
        volume_props = GProp_GProps()
        brepgprop_VolumeProperties(shape, volume_props)
        volume = round(abs(volume_props.Mass()), 2)
        
        # Calculate surface area
        surface_props = GProp_GProps()
        brepgprop_SurfaceProperties(shape, surface_props)
        surface_area = round(abs(surface_props.Mass()), 2)
        
        logger.info(f"Volume: {volume}, Surface Area: {surface_area}")
        
        # Extract features
        features = analyze_shape(shape)
        
        logger.info(f"Extracted {len(features)} feature types")
        
        return AnalyzeResponse(
            success=True,
            features=features,
            dimensions=dimensions,
            volume=volume,
            surfaceArea=surface_area
        )
        
    except Exception as e:
        logger.exception(f"Error analyzing STEP file: {e}")
        return AnalyzeResponse(success=False, error=str(e))
    
    finally:
        # Clean up temp file
        if temp_file and os.path.exists(temp_file.name):
            try:
                os.unlink(temp_file.name)
                logger.info(f"Cleaned up temp file: {temp_file.name}")
            except Exception as e:
                logger.warning(f"Failed to clean up temp file: {e}")


if __name__ == "__main__":
    import os
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=int(os.getenv("PORT", "8000")), log_level="info")





