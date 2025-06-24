let storage = require("storage");

function tiles(my_rom, ranges, tile_size, tile_bytes) {
    tile_size = tile_size || 8;
    tile_bytes = tile_bytes || 16;
    let palette = [[255,255,255], [192,192,192], [96,96,96], [0,0,0]];
    //let tiles = [];
    let count = 0;

    let file = storage.openFile(my_rom, "r", "open_existing");
    storage.makeDirectory("/ext/rom_data")
    
    for (let i = 0; i < ranges.length; i++)
    {
        let start = ranges[i][0];
        let end = ranges[i][1];
        let pos = start;

        while (pos + tile_bytes <= end + 1)
        {
            count += 1;
            file.seekAbsolute(pos);
            let raw = file.read("ascii", tile_bytes);

            let tile = [];
            let new_file = storage.openFile("/ext/rom_data/tile_" + count.toString(), "w", "create_always");


            for (let y = 0; y < tile_size; y++)
            {
                let lo = raw.charCodeAt(2 * y);
                let hi = raw.charCodeAt(2 * y + 1);
                let row = [];

                for (let x = 0; x < tile_size; x++)
                {
                    let bit = 7 - x;
                    let l = (lo >> bit) & 1;
                    let h = (hi >> bit) & 1;
                    row.push((h << 1) | l);
                    new_file.write(((h << 1) | l).toString());
                }
                tile.push(row);
                new_file.write("\n");
            }
          
            new_file.close();

            print("Extracted tile: " + count.toString());

            //tiles.push(tile);
            pos += tile_bytes;
        }
    }

    file.close();
    return tiles;
}

let tile_size = 8;
let tile_bytes = 16;
let ranges = [[0x8000, 0x87FF], [0x8800, 0x8FFF]];
let my_rom = "/ext/kirby.gb";
print("Extracting tiles.");
tiles(my_rom, ranges, tile_size, tile_bytes);

//let gui = furi_record_open("gui");
//let canvas = gui_direct_draw_acquire(gui);
