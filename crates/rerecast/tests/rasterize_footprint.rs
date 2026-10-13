//! A triangle only puts spans in the cells it covers.

use glam::{Vec3, Vec3A};
use rerecast::{Aabb3d, AreaType, HeightfieldBuilder};

/// A vertical triangle with zero width in x whose top edge lies exactly on the boundary
/// between rows 124 and 125 at 0.2 m cells. In f32 the boundary rounds so that row 124
/// leaves nothing for row 125, and `divide_poly` used to return early on that empty
/// polygon without zeroing its output counts. The caller then rasterized a stale polygon
/// stretching to the world origin: 78 spans along row 125, from x83 to x160.
#[test]
fn a_triangle_edge_on_a_cell_boundary_stays_in_its_footprint() {
    let aabb = Aabb3d {
        min: Vec3::new(-32.0, -1.0, -32.0),
        max: Vec3::new(32.0, 3.2, 32.0),
    };
    let mut heightfield = HeightfieldBuilder {
        aabb,
        cell_size: 0.2,
        cell_height: 0.1,
    }
    .build()
    .unwrap();
    let triangle = [
        Vec3A::new(-15.8, 0.0, -7.3),
        Vec3A::new(-15.8, 3.0, -7.0),
        Vec3A::new(-15.8, 0.0, -7.0),
    ];

    heightfield
        .rasterize_triangle(triangle, AreaType::NOT_WALKABLE, 4)
        .unwrap();

    let width = heightfield.width as usize;
    let outside: Vec<(usize, usize)> = (0..heightfield.height as usize)
        .flat_map(|z| (0..width).map(move |x| (x, z)))
        .filter(|&(x, z)| heightfield.spans[x + z * width].is_some() && !(80..=82).contains(&x))
        .collect();
    assert_eq!(outside, Vec::new(), "spans outside x80..=x82");
}
