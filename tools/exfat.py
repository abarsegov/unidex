# The Arduino ESP32 2.0.17 SDK is built from ESP-IDF 4.4.7. Use that same
# component's sources and the SDK's config. exFAT changes FatFs struct layouts,
# so the FatFs core, VFS adapters, and SD_MMC must all see the patched headers.
import hashlib
import shutil
import tarfile
import tempfile
import urllib.request
from pathlib import Path, PurePosixPath

Import("env")

if env.IsCleanTarget():
    Return()

idf_revision = "38eeba213aa695aabfd6d89aa9f5078dbe5a94c3"
archive_sha256 = "397194be939889b8daefae7b1d797efa1bf9adbdc79b8bf78a248216aaf65920"
archive_cache = Path(env.subst("$PROJECT_DIR")) / ".pio" / "fatfs-cache"
archive = archive_cache / f"esp-idf-{idf_revision}.tar.gz"
archive_url = f"https://codeload.github.com/espressif/esp-idf/tar.gz/{idf_revision}"
generated_dir = Path(env.subst("$BUILD_DIR")) / "fatfs-source"
include_dirs = ("diskio", "src", "vfs")
sources = {
    "diskio/diskio.c",
    "diskio/diskio_rawflash.c",
    "diskio/diskio_sdmmc.c",
    "diskio/diskio_wl.c",
    "port/freertos/ffsystem.c",
    "src/ff.c",
    "src/ffunicode.c",
    "vfs/vfs_fat.c",
    "vfs/vfs_fat_sdmmc.c",
    "vfs/vfs_fat_spiflash.c",
}

def verify_archive(path):
    if hashlib.sha256(path.read_bytes()).hexdigest() != archive_sha256:
        raise RuntimeError(f"ESP-IDF source archive checksum mismatch: {path}")


if not archive.is_file():
    archive_cache.mkdir(parents=True, exist_ok=True)
    print("Downloading the ESP-IDF 4.4.7 FatFs sources")
    with tempfile.TemporaryDirectory(dir=archive_cache) as temp_dir:
        download = Path(temp_dir) / archive.name
        with urllib.request.urlopen(archive_url, timeout=60) as response, download.open("wb") as output:
            shutil.copyfileobj(response, output)
        verify_archive(download)
        download.replace(archive)
else:
    verify_archive(archive)

fatfs_prefix = f"esp-idf-{idf_revision}/components/fatfs/"
required = sources | {"src/ff.h", "src/ffconf.h"}
with tarfile.open(archive, "r|gz") as upstream:
    for member in upstream:
        if not member.isfile() or not member.name.startswith(fatfs_prefix):
            continue
        name = member.name[len(fatfs_prefix):]
        relative_path = PurePosixPath(name)
        is_header = len(relative_path.parts) == 2 and relative_path.parts[0] in include_dirs and relative_path.suffix == ".h"
        if name not in sources and not is_header:
            continue

        with upstream.extractfile(member) as source:
            contents = source.read()
        if name == "src/ffconf.h":
            disabled = b"#define FF_FS_EXFAT\t\t0"
            if contents.count(disabled) != 1:
                raise RuntimeError("Unexpected exFAT setting in the pinned ESP-IDF FatFs source")
            contents = contents.replace(disabled, b"#define FF_FS_EXFAT\t\t1")

        target = generated_dir.joinpath(*relative_path.parts)
        if not target.is_file() or target.read_bytes() != contents:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(contents)
        required.discard(name)

if required:
    raise RuntimeError(f"Missing ESP-IDF FatFs sources: {', '.join(sorted(required))}")

env.Prepend(CPPPATH=[str(generated_dir / directory) for directory in include_dirs])
# These objects link before the SDK archives, replacing the stock libfatfs.a.
env.BuildSources("$BUILD_DIR/fatfs-exfat", str(generated_dir))
