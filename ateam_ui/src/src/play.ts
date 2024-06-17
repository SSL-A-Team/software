// Types used for plays

export class Play {
    name: string;
    score: number;
    enabled: boolean = true;
    params = {};

    constructor(name: string, enabled: boolean, score: number = NaN) {
        this.name = name;
        this.enabled = enabled;
        this.score = score;
    }
}