// Types used for plays

export class Play {
    name: string;
    score: number;
    enabled: boolean = true; // Ideally we set this up to also get feedback from software about their status

    constructor(name: string, score: number = null) {
        this.name = name;
        this.score = score;
    }
}