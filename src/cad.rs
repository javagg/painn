use truck_meshalgo::prelude::*;
use truck_modeling::*;
use truck_polymesh::PolygonMesh;
use truck_polymesh::obj::read;
use std::f64::consts::PI;

pub fn load_obj() -> PolygonMesh {
      let polygon = read(include_bytes!("teapot.obj").as_ref()).unwrap();
    return polygon;
}

pub fn to_mesh(solid: &Solid) -> PolygonMesh {
    let mesh_with_topology = solid.triangulation(0.01);
    let mesh = mesh_with_topology.to_polygon();
    mesh
}

pub fn box_solid(width: f64, height: f64, depth: f64) -> Solid {
    let vertex: Vertex = builder::vertex(Point3::new(-width / 2.0, -height / 2.0, -depth / 2.0));
    let edge: Edge = builder::tsweep(&vertex, Vector3::new(0.0, 0.0, depth));
    let face: Face = builder::tsweep(&edge, Vector3::new(width, 0.0, 0.0));
    builder::tsweep(&face, Vector3::new(0.0, height, 0.0))
}

pub fn sphere(radius: f64) -> Solid {
    let v0 = builder::vertex(Point3::new(0.0, radius, 0.0));
    let wire: Wire = builder::rsweep(&v0, Point3::origin(), Vector3::unit_x(), Rad(PI));
    let shell = builder::cone(&wire, Vector3::unit_y(), Rad(7.0));
    Solid::new(vec![shell])
}

pub fn cylinder_solid(height: f64, radius: f64) -> Solid {
    let shell = cylinder(-height / 2.0, height, radius);
    Solid::new(vec![shell])
}

pub fn cone_solid(height: f64, radius: f64) -> Solid {
    let v0 = builder::vertex(Point3::new(0.0, height / 2.0, 0.0));
    let v1 = builder::vertex(Point3::new(0.0, -height / 2.0, radius));
    let v2 = builder::vertex(Point3::new(0.0, -height / 2.0, 0.0));
    let wire: Wire = vec![builder::line(&v0, &v1), builder::line(&v1, &v2)].into();
    let shell = builder::cone(&wire, Vector3::unit_y(), Rad(7.0));
    Solid::new(vec![shell])
}

pub fn torus_solid(major: f64, minor: f64) -> Solid {
    let v = builder::vertex(Point3::new(major, 0.0, minor));
    let w = builder::rsweep(&v, Point3::new(major, 0.0, 0.0), Vector3::unit_y(), Rad(7.0));
    let shell = builder::rsweep(&w, Point3::origin(), Vector3::unit_z(), Rad(7.0));
    Solid::new(vec![shell])
}

/// Construct a cube
pub fn cube() -> Solid {
    // put a vertex at the point (-1, 0, -1)
    let vertex: Vertex = builder::vertex(Point3::new(-1.0, 0.0, -1.0));
    // sweep the vertex along the z-axis
    let edge: Edge = builder::tsweep(
        // the reference to the vertex
        &vertex,
        // sweep along the z-axis for length 2
        2.0 * Vector3::unit_z(),
    );
    // sweep the edge along the x-axis
    let face: Face = builder::tsweep(
        // the reference to the edge
        &edge,
        // sweep along the x-axis for length 2
        2.0 * Vector3::unit_x(),
    );
    // sweep the face along the y-axis
    builder::tsweep(
        // the reference to the face
        &face,
        // sweep along the y-axis for length 2
        2.0 * Vector3::unit_y(),
    )
}


// modeling a torus
pub fn torus() -> Solid {
    // put a vertex at the point (0, 0, 1).
    let vertex = builder::vertex(Point3::new(0.0, 0.0, 1.0));
    // sweep the vertex along a circle
    let circle: Wire = builder::rsweep(
        // the reference to the vertex
        &vertex,
        // a point on the axis
        Point3::new(0.0, 0.5, 1.0),
        // the direction of the axis
        Vector3::unit_x(),
        // If the absolute value is no less than 2π radian, a closed shape will be generated.
        Rad(7.0),
    );
    // sweep the circle along a circle
    let boundary = builder::rsweep(
        // the reference to the wire
        &circle,
        // a point on the axis
        Point3::origin(),
        // the direction of the axis
        Vector3::unit_y(),
        // If the absolute value is no less than 2π radian, a closed shape will be generated.
        Rad(7.0),
    );
    Solid::new(vec![boundary])
}


// modeling the body shape
// # Arguments
// - bottom: y-coordinate of the bottom face
// - height: height of the body
// - width: width of the body
// - thickness: thickness of the body
fn body_shell(bottom: f64, height: f64, width: f64, thickness: f64) -> Shell {
    // draw a circle arc
    let vertex0 = builder::vertex(Point3::new(-width / 2.0, bottom, thickness / 4.0));
    let vertex1 = builder::vertex(Point3::new(width / 2.0, bottom, thickness / 4.0));
    let transit = Point3::new(0.0, bottom, thickness / 2.0);
    let arc0 = builder::circle_arc(&vertex0, &vertex1, transit);
    // copy and rotate the circle arc
    let arc1 = builder::rotated(&arc0, Point3::origin(), Vector3::unit_y(), Rad(PI));
    // create the homotopy from arc0 to arc1.inverse()
    let face = builder::homotopy(&arc0, &arc1.inverse());
    // create the body
    let solid = builder::tsweep(&face, Vector3::new(0.0, height, 0.0));
    // Return the solid as a boundary shell for easier processing later.
    solid.into_boundaries().pop().unwrap()
}

// modeling a cylinder
// # Arguments
// - bottom: y-coordinate of the bottom disk
// - height: height of the cylinder
// - radius: radius of the bottom disk
fn cylinder(bottom: f64, height: f64, radius: f64) -> Shell {
    // make a solid cylinder
    let vertex = builder::vertex(Point3::new(0.0, bottom, radius));
    let circle = builder::rsweep(&vertex, Point3::origin(), Vector3::unit_y(), Rad(7.0));
    let disk = builder::try_attach_plane(&vec![circle]).unwrap();
    let solid = builder::tsweep(&disk, Vector3::new(0.0, height, 0.0));
    // Return the solid as a boundary shell for easier processing later.
    solid.into_boundaries().pop().unwrap()
}

// sew the body and the neck
fn glue_body_neck(body: &mut Shell, neck: Shell) {
    // get the body's ceiling
    let body_ceiling = body.last_mut().unwrap();
    // the boundary of the neck's bottom
    let wire = neck[0].boundaries()[0].clone();
    // drill a hole in the body using the boundary of the neck's bottom
    body_ceiling.add_boundary(wire);
    // add the faces of the neck to the body other than the bottom
    body.extend(neck.into_iter().skip(1));
}


// modeling a bottle
pub fn bottle(height: f64, width: f64, thickness: f64) -> Solid {
    // create the body of the bottle
    let mut body = body_shell(-height / 2.0, height, width, thickness);
    // create the neck of the bottle
    let neck = cylinder(height / 2.0, height / 10.0, thickness / 4.0);
    // sew the body and the neck
    glue_body_neck(&mut body, neck);

    // distance between outer and inner surface, i.e. the thickness of the faces.
    let eps = height / 50.0;
    // inner body. Make it small enough to account for thickness.
    let mut inner_body = body_shell(
        -height / 2.0 + eps,
        height - 2.0 * eps,
        width - 2.0 * eps,
        thickness - 2.0 * eps,
    );
    // inner neck. Make it long and narrow to account for thickness.
    let inner_neck = cylinder(
        height / 2.0 - eps,
        height / 10.0 + eps,
        thickness / 4.0 - eps,
    );
    // sew the inner body and the inner neck
    glue_body_neck(&mut inner_body, inner_neck);

    // invert all faces of the inner body
    inner_body.face_iter_mut().for_each(|face| {
        face.invert();
    });
    // pop the ceiling of the inner body
    let inner_ceiling = inner_body.pop().unwrap();
    // make the inner ceiling the boundary wire
    let wire = inner_ceiling.into_boundaries().pop().unwrap();
    // the mutable reference to the outer ceiling
    let ceiling = body.last_mut().unwrap();
    // drill a hole in the outer ceiling using the boundary of inner ceiling
    ceiling.add_boundary(wire);
    // add the faces of the neck to the body
    body.extend(inner_body.into_iter());
    // returns the solid
    Solid::new(vec![body])
}
