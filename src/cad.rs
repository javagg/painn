#![cfg_attr(target_arch = "wasm32", allow(dead_code))]

use truck_meshalgo::prelude::*;
use truck_modeling::*;
use truck_polymesh::{Faces, PolygonMesh, StandardAttributes, StandardVertex};
use truck_shapeops::or;

use meshx::mesh::TetMesh;
#[cfg(not(target_arch = "wasm32"))]
use meshx::io::msh::{self, ElementType};
#[cfg(target_arch = "wasm32")]
use mshio::{self as msh, ElementType};
use std::collections::HashMap;
use std::path::Path;
use std::f64::consts::PI;

pub fn to_mesh(solid: &Solid) -> PolygonMesh {
    let mesh_with_topology = solid.triangulation(0.01);
    let mesh = mesh_with_topology.to_polygon();
    mesh
}

pub fn load_gmsh_mesh_from_bytes(msh_bytes: &[u8]) -> std::result::Result<PolygonMesh, String> {
    let msh: msh::MshFile<u64, i32, f64> =
        msh::parse_msh_bytes(msh_bytes).map_err(|e| e.to_string())?;

    let nodes = msh
        .data
        .nodes
        .as_ref()
        .ok_or_else(|| "Missing nodes in Gmsh file".to_string())?;

    let mut positions: Vec<Point3> = Vec::new();
    let mut point_map: HashMap<u64, usize> = HashMap::new();

    let mut add_node = |node_tag: u64, node: &msh::Node<f64>| -> usize {
        *point_map.entry(node_tag).or_insert_with(|| {
            let new_index = positions.len();
            positions.push(Point3::new(node.x, node.y, node.z));
            new_index
        })
    };

    let mut find_and_add_node = |node_tag: u64| -> Option<usize> {
        let mut offset = 0;
        for block in nodes.node_blocks.iter() {
            if let Some(tags) = &block.node_tags {
                if let Some(&index_in_block) = tags.get(&node_tag) {
                    return Some(add_node(node_tag, &block.nodes[index_in_block - 1]));
                }
            } else {
                let mut node_index = node_tag as usize;
                if node_index < offset {
                    break;
                }
                node_index -= offset;
                if node_index <= block.nodes.len() {
                    return Some(add_node(node_tag, &block.nodes[node_index - 1]));
                }
            }
            offset += block.nodes.len();
        }
        None
    };

    let elements = msh
        .data
        .elements
        .as_ref()
        .ok_or_else(|| "Missing elements in Gmsh file".to_string())?;

    let mut tri_faces: Vec<[StandardVertex; 3]> = Vec::new();
    let mut tet_cells: Vec<[usize; 4]> = Vec::new();

    for block in elements.element_blocks.iter() {
        match block.element_type {
            ElementType::Tri3 => {
                for element in block.elements.iter() {
                    if element.nodes.len() < 3 {
                        continue;
                    }
                    let a = find_and_add_node(element.nodes[0])
                        .ok_or_else(|| "Missing node data".to_string())?;
                    let b = find_and_add_node(element.nodes[1])
                        .ok_or_else(|| "Missing node data".to_string())?;
                    let c = find_and_add_node(element.nodes[2])
                        .ok_or_else(|| "Missing node data".to_string())?;
                    tri_faces.push([
                        StandardVertex { pos: a, uv: None, nor: None },
                        StandardVertex { pos: b, uv: None, nor: None },
                        StandardVertex { pos: c, uv: None, nor: None },
                    ]);
                }
            }
            ElementType::Tet4 => {
                for element in block.elements.iter() {
                    if element.nodes.len() < 4 {
                        continue;
                    }
                    let a = find_and_add_node(element.nodes[0])
                        .ok_or_else(|| "Missing node data".to_string())?;
                    let b = find_and_add_node(element.nodes[1])
                        .ok_or_else(|| "Missing node data".to_string())?;
                    let c = find_and_add_node(element.nodes[2])
                        .ok_or_else(|| "Missing node data".to_string())?;
                    let d = find_and_add_node(element.nodes[3])
                        .ok_or_else(|| "Missing node data".to_string())?;
                    tet_cells.push([a, b, c, d]);
                }
            }
            _ => {}
        }
    }

    if !tet_cells.is_empty() {
        let surface = TetMesh::<f64>::surface_topo_from_tets(tet_cells.iter());
        for tri in surface {
            tri_faces.push([
                StandardVertex { pos: tri[0], uv: None, nor: None },
                StandardVertex { pos: tri[1], uv: None, nor: None },
                StandardVertex { pos: tri[2], uv: None, nor: None },
            ]);
        }
    }

    if tri_faces.is_empty() {
        return Err("No surface triangles found in the Gmsh mesh.".to_string());
    }

    let attributes = StandardAttributes {
        positions,
        uv_coords: Vec::new(),
        normals: Vec::new(),
    };
    let faces = Faces::from_tri_and_quad_faces(tri_faces, Vec::new());
    Ok(PolygonMesh::new(attributes, faces))
}

pub fn load_gmsh_mesh(path: &Path) -> std::result::Result<PolygonMesh, String> {
    let msh_bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    load_gmsh_mesh_from_bytes(msh_bytes.as_slice())
}

pub fn box_solid(width: f64, height: f64, depth: f64) -> Solid {
    let vertex: Vertex = builder::vertex(Point3::new(-width / 2.0, -height / 2.0, -depth / 2.0));
    let edge: Edge = builder::tsweep(&vertex, Vector3::new(0.0, 0.0, depth));
    let face: Face = builder::tsweep(&edge, Vector3::new(width, 0.0, 0.0));
    builder::tsweep(&face, Vector3::new(0.0, height, 0.0))
}

pub fn box_solid_at(center: [f64; 3], width: f64, height: f64, depth: f64) -> Solid {
    let (cx, cy, cz) = (center[0], center[1], center[2]);
    let vertex: Vertex =
        builder::vertex(Point3::new(cx - width / 2.0, cy - height / 2.0, cz - depth / 2.0));
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

pub fn sphere_at(center: [f64; 3], radius: f64) -> Solid {
    let (cx, cy, cz) = (center[0], center[1], center[2]);
    let v0 = builder::vertex(Point3::new(cx, cy + radius, cz));
    let wire: Wire =
        builder::rsweep(&v0, Point3::new(cx, cy, cz), Vector3::unit_x(), Rad(PI));
    let shell = builder::cone(&wire, Vector3::unit_y(), Rad(7.0));
    Solid::new(vec![shell])
}

pub fn cylinder_solid(height: f64, radius: f64) -> Solid {
    let shell = cylinder(-height / 2.0, height, radius);
    Solid::new(vec![shell])
}

pub fn cylinder_solid_at(center: [f64; 3], height: f64, radius: f64) -> Solid {
    let (cx, cy, cz) = (center[0], center[1], center[2]);
    let bottom = cy - height / 2.0;
    let vertex = builder::vertex(Point3::new(cx, bottom, cz + radius));
    let circle = builder::rsweep(&vertex, Point3::new(cx, bottom, cz), Vector3::unit_y(), Rad(7.0));
    let disk = builder::try_attach_plane(&vec![circle]).unwrap();
    let solid = builder::tsweep(&disk, Vector3::new(0.0, height, 0.0));
    Solid::new(vec![solid.into_boundaries().pop().unwrap()])
}

pub fn cone_solid(height: f64, base_radius: f64, top_radius: f64) -> Solid {
    if top_radius <= 1.0e-6 {
        let v0 = builder::vertex(Point3::new(0.0, height / 2.0, 0.0));
        let v1 = builder::vertex(Point3::new(0.0, -height / 2.0, base_radius));
        let v2 = builder::vertex(Point3::new(0.0, -height / 2.0, 0.0));
        let wire: Wire = vec![builder::line(&v0, &v1), builder::line(&v1, &v2)].into();
        let shell = builder::cone(&wire, Vector3::unit_y(), Rad(7.0));
        return Solid::new(vec![shell]);
    }

    let bottom_center = Point3::new(0.0, -height / 2.0, 0.0);
    let top_center = Point3::new(0.0, height / 2.0, 0.0);
    let bottom_vertex = builder::vertex(Point3::new(0.0, -height / 2.0, base_radius));
    let top_vertex = builder::vertex(Point3::new(0.0, height / 2.0, top_radius));
    let bottom_circle = builder::rsweep(&bottom_vertex, bottom_center, Vector3::unit_y(), Rad(7.0));
    let top_circle = builder::rsweep(&top_vertex, top_center, Vector3::unit_y(), Rad(7.0));
    let mut shell = builder::try_wire_homotopy(&bottom_circle, &top_circle).unwrap();

    if let Ok(bottom_face) = builder::try_attach_plane(&vec![bottom_circle.clone()]) {
        shell.push(bottom_face);
    }
    if let Ok(top_face) = builder::try_attach_plane(&vec![top_circle.clone()]) {
        shell.push(top_face);
    }

    Solid::new(vec![shell])
}

pub fn cone_solid_at(center: [f64; 3], height: f64, base_radius: f64, top_radius: f64) -> Solid {
    let (cx, cy, cz) = (center[0], center[1], center[2]);

    if top_radius <= 1.0e-6 {
        let v0 = builder::vertex(Point3::new(cx, cy + height / 2.0, cz));
        let v1 = builder::vertex(Point3::new(cx, cy - height / 2.0, cz + base_radius));
        let v2 = builder::vertex(Point3::new(cx, cy - height / 2.0, cz));
        let wire: Wire = vec![builder::line(&v0, &v1), builder::line(&v1, &v2)].into();
        let shell = builder::cone(&wire, Vector3::unit_y(), Rad(7.0));
        return Solid::new(vec![shell]);
    }

    let bottom_center = Point3::new(cx, cy - height / 2.0, cz);
    let top_center = Point3::new(cx, cy + height / 2.0, cz);
    let bottom_vertex = builder::vertex(Point3::new(cx, cy - height / 2.0, cz + base_radius));
    let top_vertex = builder::vertex(Point3::new(cx, cy + height / 2.0, cz + top_radius));
    let bottom_circle = builder::rsweep(&bottom_vertex, bottom_center, Vector3::unit_y(), Rad(7.0));
    let top_circle = builder::rsweep(&top_vertex, top_center, Vector3::unit_y(), Rad(7.0));
    let mut shell = builder::try_wire_homotopy(&bottom_circle, &top_circle).unwrap();

    if let Ok(bottom_face) = builder::try_attach_plane(&vec![bottom_circle.clone()]) {
        shell.push(bottom_face);
    }
    if let Ok(top_face) = builder::try_attach_plane(&vec![top_circle.clone()]) {
        shell.push(top_face);
    }

    Solid::new(vec![shell])
}

pub fn torus_solid(major: f64, minor: f64) -> Solid {
    let v = builder::vertex(Point3::new(major, 0.0, minor));
    let w = builder::rsweep(&v, Point3::new(major, 0.0, 0.0), Vector3::unit_y(), Rad(7.0));
    let shell = builder::rsweep(&w, Point3::origin(), Vector3::unit_z(), Rad(7.0));
    Solid::new(vec![shell])
}

pub fn torus_solid_at(center: [f64; 3], major: f64, minor: f64) -> Solid {
    let (cx, cy, cz) = (center[0], center[1], center[2]);
    let v = builder::vertex(Point3::new(cx + major, cy, cz + minor));
    let w = builder::rsweep(
        &v,
        Point3::new(cx + major, cy, cz),
        Vector3::unit_y(),
        Rad(7.0),
    );
    let shell = builder::rsweep(&w, Point3::new(cx, cy, cz), Vector3::unit_z(), Rad(7.0));
    Solid::new(vec![shell])
}

/// Construct a cube
// #[allow(dead_code)]
// pub fn cube() -> Solid {
//     // put a vertex at the point (-1, 0, -1)
//     let vertex: Vertex = builder::vertex(Point3::new(-1.0, 0.0, -1.0));
//     // sweep the vertex along the z-axis
//     let edge: Edge = builder::tsweep(
//         // the reference to the vertex
//         &vertex,
//         // sweep along the z-axis for length 2
//         2.0 * Vector3::unit_z(),
//     );
//     // sweep the edge along the x-axis
//     let face: Face = builder::tsweep(
//         // the reference to the edge
//         &edge,
//         // sweep along the x-axis for length 2
//         2.0 * Vector3::unit_x(),
//     );
//     // sweep the face along the y-axis
//     builder::tsweep(
//         // the reference to the face
//         &face,
//         // sweep along the y-axis for length 2
//         2.0 * Vector3::unit_y(),
//     )
// }


// modeling a torus
// #[allow(dead_code)]
// pub fn torus() -> Solid {
//     // put a vertex at the point (0, 0, 1).
//     let vertex = builder::vertex(Point3::new(0.0, 0.0, 1.0));
//     // sweep the vertex along a circle
//     let circle: Wire = builder::rsweep(
//         // the reference to the vertex
//         &vertex,
//         // a point on the axis
//         Point3::new(0.0, 0.5, 1.0),
//         // the direction of the axis
//         Vector3::unit_x(),
//         // If the absolute value is no less than 2π radian, a closed shape will be generated.
//         Rad(7.0),
//     );
//     // sweep the circle along a circle
//     let boundary = builder::rsweep(
//         // the reference to the wire
//         &circle,
//         // a point on the axis
//         Point3::origin(),
//         // the direction of the axis
//         Vector3::unit_y(),
//         // If the absolute value is no less than 2π radian, a closed shape will be generated.
//         Rad(7.0),
//     );
//     Solid::new(vec![boundary])
// }


// modeling the body shape
// # Arguments
// - bottom: y-coordinate of the bottom face
// - height: height of the body
// - width: width of the body
// - thickness: thickness of the body
// #[allow(dead_code)]
// fn body_shell(bottom: f64, height: f64, width: f64, thickness: f64) -> Shell {
//     // draw a circle arc
//     let vertex0 = builder::vertex(Point3::new(-width / 2.0, bottom, thickness / 4.0));
//     let vertex1 = builder::vertex(Point3::new(width / 2.0, bottom, thickness / 4.0));
//     let transit = Point3::new(0.0, bottom, thickness / 2.0);
//     let arc0 = builder::circle_arc(&vertex0, &vertex1, transit);
//     // copy and rotate the circle arc
//     let arc1 = builder::rotated(&arc0, Point3::origin(), Vector3::unit_y(), Rad(PI));
//     // create the homotopy from arc0 to arc1.inverse()
//     let face = builder::homotopy(&arc0, &arc1.inverse());
//     // create the body
//     let solid = builder::tsweep(&face, Vector3::new(0.0, height, 0.0));
//     // Return the solid as a boundary shell for easier processing later.
//     solid.into_boundaries().pop().unwrap()
// }

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
#[allow(dead_code)]
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
// #[allow(dead_code)]
// pub fn bottle(height: f64, width: f64, thickness: f64) -> Solid {
//     // create the body of the bottle
//     let mut body = body_shell(-height / 2.0, height, width, thickness);
//     // create the neck of the bottle
//     let neck = cylinder(height / 2.0, height / 10.0, thickness / 4.0);
//     // sew the body and the neck
//     glue_body_neck(&mut body, neck);

//     // distance between outer and inner surface, i.e. the thickness of the faces.
//     let eps = height / 50.0;
//     // inner body. Make it small enough to account for thickness.
//     let mut inner_body = body_shell(
//         -height / 2.0 + eps,
//         height - 2.0 * eps,
//         width - 2.0 * eps,
//         thickness - 2.0 * eps,
//     );
//     // inner neck. Make it long and narrow to account for thickness.
//     let inner_neck = cylinder(
//         height / 2.0 - eps,
//         height / 10.0 + eps,
//         thickness / 4.0 - eps,
//     );
//     // sew the inner body and the inner neck
//     glue_body_neck(&mut inner_body, inner_neck);

//     // invert all faces of the inner body
//     inner_body.face_iter_mut().for_each(|face| {
//         face.invert();
//     });
//     // pop the ceiling of the inner body
//     let inner_ceiling = inner_body.pop().unwrap();
//     // make the inner ceiling the boundary wire
//     let wire = inner_ceiling.into_boundaries().pop().unwrap();
//     // the mutable reference to the outer ceiling
//     let ceiling = body.last_mut().unwrap();
//     // drill a hole in the outer ceiling using the boundary of inner ceiling
//     ceiling.add_boundary(wire);
//     // add the faces of the neck to the body
//     body.extend(inner_body.into_iter());
//     // returns the solid
//     Solid::new(vec![body])
// }
pub fn solid_unite(solids: &[Solid]) -> Solid {
    let Some((first, rest)) = solids.split_first() else {
        return Solid::new(Vec::new());
    };

    let mut result = first.clone();
    for solid in rest {
        if let Some(next) = or(&result, solid, 1.0e-6) {
            result = next;
        }
    }
    result
}