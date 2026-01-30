use truck_polymesh::PolygonMesh;
use truck_polymesh::obj::read;

pub fn load_obj() -> PolygonMesh {
      let polygon = read(include_bytes!("teapot.obj").as_ref()).unwrap();
    return polygon;
}