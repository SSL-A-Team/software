// Types used for plays

export class Play {
    name: string;
    score: number;
    enabled: boolean = true; // Ideally we set this up to also get feedback from software about their status

    constructor(name: string, enabled:boolean, score: number = NaN) {
        this.name = name;
        this.enabled = enabled;
        this.score = score;
    }
}