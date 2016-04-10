$fn = 100;
screw_thickness = 2;
screw_height = 4;
mount_screw_d = 4;
mount_x_size = 107;
mount_y_size = 39.25;
mount_z_size = 52;
plate_x_size = 2;
plate_y_size = 70;
sensor_mount_y = 70;
plate_thickness = 2;
sensor_height = 10;
sensor_size = 61.9;
bar_thickness = 8;

module screw_hole(x, y, z, screw_r=mount_screw_d / 2, wall_r=screw_thickness, height=screw_height)
{
    difference()
    {
        translate([x - (screw_r + wall_r), y - (screw_r + wall_r), z - height / 2])
        cube([2 * (screw_r + wall_r), 2 * (screw_r + wall_r), height], center=false);

        translate([x, y - 1.4 * screw_r, z - height], center=false)
        rotate([0, 0, 45])
        cube([2 * screw_r, 2 * screw_r, 2 * height], center=false);
    }
}

module screw_mount(x)
{
    difference()
    {
        translate([x - 0.5 * mount_screw_d - screw_thickness, -plate_y_size / 2, -screw_height / 2])
        cube([mount_screw_d + 2 * screw_thickness, plate_y_size, screw_height],
            center=false);

        translate([x, mount_y_size / 2 - 0.7 * mount_screw_d, -screw_height], center=false)
        rotate([0, 0, 45])
        cube([mount_screw_d, mount_screw_d, 2 * screw_height], center=false);

        translate([x, -mount_y_size / 2 - 0.7 * mount_screw_d, -screw_height], center=false)
        rotate([0, 0, 45])
        cube([mount_screw_d, mount_screw_d, 2 * screw_height], center=false);

    }
}

screw_mount(0);

translate([-0.5 * mount_screw_d - screw_thickness, -plate_y_size / 2, 0])
cube([mount_screw_d + 2 * screw_thickness, bar_thickness, mount_z_size + 2]);

translate([-0.5 * mount_screw_d - screw_thickness, (plate_y_size - 2 * mount_screw_d - screw_thickness - bar_thickness) / 2 + 1, 0])
cube([mount_screw_d + 2 * screw_thickness, bar_thickness, mount_z_size + 2]);

for (j = [-1, 1])
{
    screw_hole(mount_x_size / 2 - sensor_size / 2, j * sensor_size / 2, mount_z_size);
    translate([-4, j * sensor_size / 2 - 4, mount_z_size - 2])
    cube([23, mount_screw_d + 2 * screw_thickness, screw_height], center=false);
}
