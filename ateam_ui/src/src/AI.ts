// Types used for ai data including what it thinks the referee states are and the current intended behavior


export class AIState {
    name: string;
    description: string;

    constructor() {
        this.name = "No Play Specified";
        this.description = '{"No Play Specified":""}'
    }
}

