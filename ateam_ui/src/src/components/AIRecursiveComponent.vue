<template>
        {{ this.name }}
        <div v-if="this.isLeaf" :style="{'margin-left': this.spacing + 'px'}">
            {{ this.node }}
        </div>
        <div v-if="!this.isLeaf" :style="{'margin-left': this.spacing + 'px'}">
            <AIRecursiveComponent
                v-for="member in Object.getOwnPropertyNames(this.node)"
                    :key= "member"
                    :name= "member"
                    :node= "this.node[member]"
                    :spacing= "this.spacing + 5"
                    style="whitespace: nowrap"
            />
        </div>
</template>

<script lang="ts">
import { props } from "vue";

export default {
    props: {
        node: {
            type: Object,
            required: true
        },
        name: {
            type: String,
            required: true
        },
        spacing: {
            type: Number,
            default: 5
        }
    },
    computed: {
        isLeaf: function() {
            return (typeof this.node === "string" || typeof this.node=== "number" || typeof this.node === "boolean")
        },
        getMargin: function() {
            return "margin-left: " + this.spacing + "px";
        }
    }
}
</script>