const std = @import("std");
const Target = std.Target;
const CrossTarget = std.zig.CrossTarget;
const Feature = std.Target.Cpu.Feature;

pub fn build(b: *std.Build) void {
    const arm_cm4_target = CrossTarget{
        .cpu_arch = .thumb,
        .os_tag = .freestanding,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
        .abi = .eabihf,
    };
    const resolver_target = b.resolveTargetQuery(arm_cm4_target);

    const elf = b.addExecutable(.{
        .name = "stm32f411.elf",
        .target = resolver_target,
        .root_source_file = .{ .path = "src/_init.zig" },
    });
    elf.setLinkerScript(.{ .path = "src/linker.ld" });

    const copy_elf = b.addInstallArtifact(elf, .{});
    const bin = b.addObjCopy(elf.getEmittedBin(), .{ .format = .bin });
    const copy_bin = b.addInstallBinFile(bin.getOutput(), "stm32f411.bin");

    bin.step.dependOn(&copy_elf.step);
    copy_bin.step.dependOn(&bin.step);

    b.default_step.dependOn(&copy_bin.step);
}
