const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .ReleaseSafe });

    var cpu_features = std.Target.Cpu.Feature.Set.empty;
    cpu_features.addFeature(@intFromEnum(std.Target.arm.Feature.vfp4d16sp));
    cpu_features.addFeature(@intFromEnum(std.Target.arm.Feature.loop_align));
    cpu_features.addFeature(@intFromEnum(std.Target.arm.Feature.no_branch_predictor));
    cpu_features.addFeature(@intFromEnum(std.Target.arm.Feature.use_misched));
    cpu_features.addFeature(@intFromEnum(std.Target.arm.Feature.v7em));

    const arm_cm4_target = std.Target.Query{
        .cpu_arch = .thumb,
        .os_tag = .freestanding,
        .cpu_model = .{
            .explicit = &.{
                .name = "cortex_m4",
                .llvm_name = "cortex-m4",
                .features = .{ .ints = .{0} ** 5 },
            },
        },
        .abi = .eabihf,
        .cpu_features_add = cpu_features,
    };

    const target = b.resolveTargetQuery(arm_cm4_target);

    const elf = b.addExecutable(.{
        .name = "stm32f411.elf",
        .target = target,
        .root_source_file = .{ .src_path = .{ .sub_path = "src/_init.zig", .owner = b } },
        .optimize = optimize,
        .strip = false,
    });
    const flash_init = b.addObject(.{
        .target = target,
        .name = "flash_init",
        .root_source_file = .{ .src_path = .{ .sub_path = "src/flash_init.zig", .owner = b } },
        .optimize = optimize,
        .strip = false,
    });
    elf.addObject(flash_init);
    elf.setLinkerScript(.{ .src_path = .{ .sub_path = "linker.ld", .owner = b } });

    const copy_elf = b.addInstallArtifact(elf, .{});

    b.default_step.dependOn(&copy_elf.step);
}
