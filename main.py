from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import base64
import tempfile
import os

app = FastAPI()

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

class Feature(BaseModel):
    id: str
    type: str
    parameters: dict
    confidence: float

class AnalyzeResponse(BaseModel):
    features: list[Feature]
    dimensions: dict
    volume: float
    surfaceArea: float

@app.post("/analyze", response_model=AnalyzeResponse)
async def analyze_step(request: AnalyzeRequest):
    try:
        from OCC.Core.STEPControl import STEPControl_Reader
        from OCC.Core.TopExp import TopExp_Explorer
        from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE
        from OCC.Core.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
        from OCC.Core.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Circle
        from OCC.Core.TopoDS import topods
        from OCC.Core.Bnd import Bnd_Box
        from OCC.Core.BRepBndLib import brepbndlib
        from OCC.Core.GProp import GProp_GProps
        from OCC.Core.BRepGProp import brepgprop

        file_data = base64.b64decode(request.fileBase64)
        
        with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as tmp:
            tmp.write(file_data)
            tmp_path = tmp.name

        reader = STEPControl_Reader()
        status = reader.ReadFile(tmp_path)
        os.unlink(tmp_path)

        if status != 1:
            raise HTTPException(status_code=400, detail="Failed to read STEP file")

        reader.TransferRoots()
        shape = reader.OneShape()

        features = []
        feature_id = 0

        face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while face_explorer.More():
            face = topods.Face(face_explorer.Current())
            surface = BRepAdaptor_Surface(face)
            
            if surface.GetType() == GeomAbs_Cylinder:
                cylinder = surface.Cylinder()
                features.append(Feature(
                    id=f"hole_{feature_id}",
                    type="hole",
                    parameters={"diameter": cylinder.Radius() * 2, "radius": cylinder.Radius()},
                    confidence=0.9
                ))
                feature_id += 1
            elif surface.GetType() == GeomAbs_Plane:
                props = GProp_GProps()
                brepgprop.SurfaceProperties(face, props)
                area = props.Mass()
                if area > 100:
                    features.append(Feature(
                        id=f"pocket_{feature_id}",
                        type="pocket",
                        parameters={"area": area},
                        confidence=0.7
                    ))
                    feature_id += 1
            face_explorer.Next()

        edge_explorer = TopExp_Explorer(shape, TopAbs_EDGE)
        while edge_explorer.More():
            edge = topods.Edge(edge_explorer.Current())
            curve = BRepAdaptor_Curve(edge)
            if curve.GetType() == GeomAbs_Circle:
                circle = curve.Circle()
                features.append(Feature(
                    id=f"fillet_{feature_id}",
                    type="fillet",
                    parameters={"radius": circle.Radius()},
                    confidence=0.85
                ))
                feature_id += 1
            edge_explorer.Next()

        bbox = Bnd_Box()
        brepbndlib.Add(shape, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()

        vol_props = GProp_GProps()
        brepgprop.VolumeProperties(shape, vol_props)
        volume = vol_props.Mass()

        surf_props = GProp_GProps()
        brepgprop.SurfaceProperties(shape, surf_props)
        surface_area = surf_props.Mass()

        return AnalyzeResponse(
            features=features,
            dimensions={"length": xmax - xmin, "width": ymax - ymin, "height": zmax - zmin},
            volume=volume,
            surfaceArea=surface_area
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health():
    return {"status": "ok"}
