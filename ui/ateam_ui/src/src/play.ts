// Types used for plays

export class Play {
    name: string;
    score: number;
    enabled: boolean = true; // Ideally we set this up to also get feedback from software about their status

    constructor(name: string, enabled:boolean, score: number = NaN) {
        this.name = name;
        this.score = score;
        this.enabled = enabled;
    }
}

export function playMapToCompressedArray(playMap: Map<string, Play>, playNameCache: string[]): number[] {

    let compressedArray: number[] = [];

    for (const [name, play] of playMap) {
        let playIndex = playNameCache.indexOf(name);

        if (playIndex == -1) {
            playNameCache.push(name);
            playIndex = playNameCache.length - 1;
        }

        compressedArray.push(playIndex);
        compressedArray.push(Number(play.enabled));
        compressedArray.push(play.score);
    }

    return compressedArray;
}

export function compressedArrayToPlayMap(compressedArray: number[], playNameCache: string[]): Map<string, Play> {

    let playMap: Map<string, Play> = new Map();

    for (let i = 0; i < compressedArray.length; i += 3) {
        playMap.set(playNameCache[compressedArray[i]],
            new Play(
                playNameCache[compressedArray[i]],
                Boolean(compressedArray[i+1]),
                compressedArray[i+2],
        ));
    }

    return playMap;
}