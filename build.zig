const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .ReleaseSmall });

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
    });
    const flash_init = b.addObject(.{
        .target = target,
        .name = "flash_init",
        .root_source_file = .{ .src_path = .{ .sub_path = "src/flash_init.zig", .owner = b } },
        .optimize = optimize,
    });
    elf.addObject(flash_init);
    elf.setLinkerScript(.{ .src_path = .{ .sub_path = "linker.ld", .owner = b } });

    const copy_elf = b.addInstallArtifact(elf, .{});

    const sysram_elf = b.addExecutable(.{
        .name = "sysram-elf",
        .target = target,
        .root_source_file = .{ .src_path = .{ .sub_path = "src/_init.zig", .owner = b } },
        .optimize = optimize,
    });

    const sysram_init = b.addObject(.{
        .target = target,
        .name = "sysram_init",
        .root_source_file = .{ .src_path = .{ .sub_path = "src/sysram_init.zig", .owner = b } },
        .optimize = optimize,
    });
    sysram_elf.addObject(sysram_init);
    sysram_elf.setLinkerScript(.{ .src_path = .{ .sub_path = "sysram.ld", .owner = b } });

    const copy_sram_elf = b.addInstallArtifact(sysram_elf, .{});
    copy_sram_elf.step.dependOn(&copy_elf.step);

    b.default_step.dependOn(&copy_sram_elf.step);
}
