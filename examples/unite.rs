use pain::cad::*;

fn main() {
    println!("Testing unite operation...");
    let solid1 = cylinder_solid_at([0.0, 0.0, 0.0],  2.0, 1.0);
    let solid2 = cylinder_solid_at([1.0, 0.0, 0.0], 2.0, 1.5);
    let united_solid = solid_unite(&[solid1, solid2]);

    save_shape(&united_solid, "united_solid");
    assert!(united_solid.is_geometric_consistent());
}