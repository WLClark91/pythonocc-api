# main.py - Complete pythonOCC Feature Extraction API with Location Data
import os
import base64
import tempfile
import logging
from typing import Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import OCC modules at startup to fail fast if environment is broken
try:
    from OCC.Core.STEPControl import STEPControl_Reader
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import (
        TopAbs_FACE, TopAbs_EDGE, TopAbs_SOLID
    )
    from OCC.Core.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
    from OCC.Core.GeomAbs import (
        GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Cone,
        GeomAbs_Torus, GeomAbs_Sphere, GeomAbs_Circle
    )
    from OCC.Core.BRepGProp import brepgprop_SurfaceProperties, brepgprop_VolumeProperties
    from OCC.Core.GProp import GProp_GProps
    from OCC.Core.TopoDS import topods_Face, topods_Solid
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib_Add
    from OCC.Core.gp import gp_Pnt
    logger.info("OCC modules imported successfully")
except ImportError as e:
    logger.error(f"Failed to import OCC modules: {e}")
    raise

app = FastAPI(title="PythonOCC Feature Extraction API")

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

class ExtractedFeature(BaseModel):
    type: str
    count: int
    parameters: dict
    location: Optional[FeatureLocation] = None

class Dimensions(BaseModel):
    x: float
    y: float
    z: float

class AnalyzeResponse(BaseModel):
    success: bool
    features: Optional[list[ExtractedFeature]] = None
    dimensions: Optional[Dimensions] = None
    volume: Optional[float] = None
    surfaceArea: Optional[float] = None
    complexityScore: Optional[float] = None
    error: Optional[str] = None


def get_face_centroid(face) -> Optional[FeatureLocation]:
    """Calculate the center of mass (centroid) of a face using GProp."""
    try:
        props = GProp_GProps()
        brepgprop_SurfaceProperties(face, props)
        center = props.CentreOfMass()
        return FeatureLocation(
            x=round(center.X(), 4),
            y=round(center.Y(), 4),
            z=round(center.Z(), 4)
        )
    except Exception as e:
        logger.warning(f"Failed to get face centroid: {e}")
        return None


def get_solid_centroid(solid) -> Optional[FeatureLocation]:
    """Calculate the center of mass of a solid."""
    try:
        props = GProp_GProps()
        brepgprop_VolumeProperties(solid, props)
        center = props.CentreOfMass()
        return FeatureLocation(
            x=round(center.X(), 4),
            y=round(center.Y(), 4),
            z=round(center.Z(), 4)
        )
    except Exception as e:
        logger.warning(f"Failed to get solid centroid: {e}")
        return None


def analyze_step_file(file_path: str) -> dict:
    """Analyze a STEP file and extract features with locations."""
    
    # Read STEP file
    reader = STEPControl_Reader()
    status = reader.ReadFile(file_path)
    
    if status != IFSelect_RetDone:
        raise ValueError("Failed to read STEP file")
    
    reader.TransferRoots()
    shape = reader.OneShape()
    
    if shape.IsNull():
        raise ValueError("No shape found in STEP file")
    
    # Calculate bounding box for dimensions
    bbox = Bnd_Box()
    brepbndlib_Add(shape, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    
    dimensions = Dimensions(
        x=round(xmax - xmin, 2),
        y=round(ymax - ymin, 2),
        z=round(zmax - zmin, 2)
    )
    
    # Calculate volume and surface area
    vol_props = GProp_GProps()
    brepgprop_VolumeProperties(shape, vol_props)
    volume = round(abs(vol_props.Mass()), 2)
    
    surf_props = GProp_GProps()
    brepgprop_SurfaceProperties(shape, surf_props)
    surface_area = round(surf_props.Mass(), 2)
    
    # Feature detection with location tracking
    features_data = {
        'hole': {'count': 0, 'radii': [], 'locations': []},
        'pocket': {'count': 0, 'depths': [], 'locations': []},
        'fillet': {'count': 0, 'radii': [], 'locations': []},
        'chamfer': {'count': 0, 'angles': [], 'locations': []},
        'boss': {'count': 0, 'heights': [], 'locations': []},
        'slot': {'count': 0, 'widths': [], 'locations': []},
        'thread': {'count': 0, 'diameters': [], 'locations': []},
    }
    
    # Analyze faces
    face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_count = 0
    cylindrical_faces = []
    planar_faces = []
    
    while face_explorer.More():
        face = topods_Face(face_explorer.Current())
        face_count += 1
        
        try:
            adaptor = BRepAdaptor_Surface(face)
            surface_type = adaptor.GetType()
            
            # HOLES: Cylindrical faces (internal cylinders)
            if surface_type == GeomAbs_Cylinder:
                cylinder = adaptor.Cylinder()
                radius = round(cylinder.Radius(), 4)
                
                # Get centroid of this cylindrical face
                centroid = get_face_centroid(face)
                
                # Check if it's likely a hole (smaller radius, internal)
                if radius < min(dimensions.x, dimensions.y, dimensions.z) * 0.3:
                    features_data['hole']['count'] += 1
                    features_data['hole']['radii'].append(radius)
                    if centroid:
                        features_data['hole']['locations'].append(centroid)
                
                cylindrical_faces.append({
                    'radius': radius,
                    'location': centroid
                })
            
            # POCKETS: Planar faces (potential pocket bottoms)
            elif surface_type == GeomAbs_Plane:
                centroid = get_face_centroid(face)
                planar_faces.append({
                    'location': centroid
                })
            
            # FILLETS: Toroidal faces (rounded edges)
            elif surface_type == GeomAbs_Torus:
                torus = adaptor.Torus()
                minor_radius = round(torus.MinorRadius(), 4)
                centroid = get_face_centroid(face)
                
                features_data['fillet']['count'] += 1
                features_data['fillet']['radii'].append(minor_radius)
                if centroid:
                    features_data['fillet']['locations'].append(centroid)
            
            # CHAMFERS: Conical faces
            elif surface_type == GeomAbs_Cone:
                cone = adaptor.Cone()
                angle = round(abs(cone.SemiAngle()) * 180 / 3.14159, 1)
                centroid = get_face_centroid(face)
                
                features_data['chamfer']['count'] += 1
                features_data['chamfer']['angles'].append(angle)
                if centroid:
                    features_data['chamfer']['locations'].append(centroid)
            
            # BOSSES: Spherical faces (dome features)
            elif surface_type == GeomAbs_Sphere:
                sphere = adaptor.Sphere()
                radius = round(sphere.Radius(), 4)
                centroid = get_face_centroid(face)
                
                features_data['boss']['count'] += 1
                features_data['boss']['heights'].append(radius)
                if centroid:
                    features_data['boss']['locations'].append(centroid)
                    
        except Exception as e:
            logger.warning(f"Error analyzing face: {e}")
            continue
        
        face_explorer.Next()
    
    # Detect pockets from planar face clusters
    # Simple heuristic: internal planar faces that aren't on the bounding box
    tolerance = 0.1
    for pf in planar_faces:
        if pf['location']:
            loc = pf['location']
            # Check if face is internal (not on bounding box boundary)
            is_internal = (
                (loc.x > xmin + tolerance and loc.x < xmax - tolerance) or
                (loc.y > ymin + tolerance and loc.y < ymax - tolerance) or
                (loc.z > zmin + tolerance and loc.z < zmax - tolerance)
            )
            if is_internal:
                features_data['pocket']['count'] += 1
                features_data['pocket']['locations'].append(pf['location'])
    
    # Limit pocket detection to reasonable count
    if features_data['pocket']['count'] > 20:
        features_data['pocket']['count'] = features_data['pocket']['count'] // 4
        features_data['pocket']['locations'] = features_data['pocket']['locations'][:features_data['pocket']['count']]
    
    # Analyze edges for threads (circular edges on cylindrical faces)
    edge_explorer = TopExp_Explorer(shape, TopAbs_EDGE)
    circular_edge_count = 0
    
    while edge_explorer.More():
        edge = edge_explorer.Current()
        try:
            adaptor = BRepAdaptor_Curve(edge)
            if adaptor.GetType() == GeomAbs_Circle:
                circle = adaptor.Circle()
                radius = round(circle.Radius(), 4)
                center = circle.Location()
                
                # Threads are typically on smaller cylindrical features
                if radius < min(dimensions.x, dimensions.y) * 0.15:
                    circular_edge_count += 1
                    if circular_edge_count <= 5:  # Limit thread detection
                        features_data['thread']['diameters'].append(radius * 2)
                        features_data['thread']['locations'].append(FeatureLocation(
                            x=round(center.X(), 4),
                            y=round(center.Y(), 4),
                            z=round(center.Z(), 4)
                        ))
        except:
            pass
        edge_explorer.Next()
    
    # Estimate thread count from circular edges
    if circular_edge_count > 10:
        features_data['thread']['count'] = circular_edge_count // 8
    
    # Build features list with averaged/representative locations
    features = []
    
    for feature_type, data in features_data.items():
        if data['count'] > 0:
            # Get representative location (first one, or average)
            location = None
            if data['locations']:
                if len(data['locations']) == 1:
                    location = data['locations'][0]
                else:
                    # Use centroid of all feature locations
                    avg_x = sum(loc.x for loc in data['locations']) / len(data['locations'])
                    avg_y = sum(loc.y for loc in data['locations']) / len(data['locations'])
                    avg_z = sum(loc.z for loc in data['locations']) / len(data['locations'])
                    location = FeatureLocation(
                        x=round(avg_x, 4),
                        y=round(avg_y, 4),
                        z=round(avg_z, 4)
                    )
            
            # Build parameters
            parameters = {}
            if feature_type == 'hole' and data['radii']:
                parameters['diameter'] = round(min(data['radii']) * 2, 2)
                parameters['max_diameter'] = round(max(data['radii']) * 2, 2)
            elif feature_type == 'fillet' and data['radii']:
                parameters['radius'] = round(sum(data['radii']) / len(data['radii']), 2)
            elif feature_type == 'chamfer' and data['angles']:
                parameters['angle'] = round(sum(data['angles']) / len(data['angles']), 1)
            elif feature_type == 'boss' and data['heights']:
                parameters['height'] = round(sum(data['heights']) / len(data['heights']), 2)
            elif feature_type == 'pocket' and data['depths']:
                parameters['depth'] = round(sum(data['depths']) / len(data['depths']), 2)
            elif feature_type == 'thread' and data['diameters']:
                parameters['diameter'] = round(sum(data['diameters']) / len(data['diameters']), 2)
            
            features.append(ExtractedFeature(
                type=feature_type,
                count=data['count'],
                parameters=parameters,
                location=location
            ))
    
    # Calculate complexity score
    complexity_score = (
        face_count * 0.5 +
        features_data['hole']['count'] * 2 +
        features_data['pocket']['count'] * 3 +
        features_data['fillet']['count'] * 1 +
        features_data['chamfer']['count'] * 1.5 +
        features_data['thread']['count'] * 4 +
        features_data['boss']['count'] * 2
    )
    
    return {
        'features': features,
        'dimensions': dimensions,
        'volume': volume,
        'surfaceArea': surface_area,
        'complexityScore': round(min(complexity_score, 100), 1)
    }


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy", "occ_available": True}


@app.post("/analyze", response_model=AnalyzeResponse)
async def analyze_step(request: AnalyzeRequest):
    """Analyze a STEP file and extract features with precise locations."""
    try:
        # Decode base64 file
        file_bytes = base64.b64decode(request.fileBase64)
        
        # Write to temp file
        with tempfile.NamedTemporaryFile(suffix='.step', delete=False) as tmp:
            tmp.write(file_bytes)
            tmp_path = tmp.name
        
        try:
            # Analyze the file
            result = analyze_step_file(tmp_path)
            
            return AnalyzeResponse(
                success=True,
                features=result['features'],
                dimensions=result['dimensions'],
                volume=result['volume'],
                surfaceArea=result['surfaceArea'],
                complexityScore=result['complexityScore']
            )
        finally:
            # Clean up temp file
            os.unlink(tmp_path)
            
    except Exception as e:
        logger.error(f"Analysis failed: {e}")
        return AnalyzeResponse(
            success=False,
            error=str(e)
        )


if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)



