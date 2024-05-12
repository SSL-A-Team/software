<template>
        {{ this.name }}

        <div v-if="this.isLeaf" :style="{'margin-left': this.spacing + 'px'}">
            <!-- Leaf: String/Number/Bool -->
            {{ this.node }}
        </div>

        <div v-if="this.isArray" :style="{'margin-left': this.spacing + 'px'}">
            <!-- Array -->
            <AIRecursiveComponent
                v-for="item in this.node"
                    :key= "item"
                    :name= "''"
                    :node= "item"
                    :spacing= "this.spacing + 5"
                    style="white-space: normal"
            />
        </div>

        <div v-if="!this.isArray && !this.isLeaf && this.isValid" :style="{'margin-left': this.spacing + 'px'}">
            <!-- Recursive Object -->
            <AIRecursiveComponent
                v-for="member in Object.getOwnPropertyNames(this.node)"
                    :key= "member"
                    :name= "member"
                    :node= "this.node[member]"
                    :spacing= "this.spacing + 5"
                    style="white-space: normal"
            />
        </div>
</template>

<script lang="ts">

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
        isValid: function() {
            return !(this.node === null || this.node === undefined)
        },
        isLeaf: function() {
            return !Array.isArray(this.node) &&
                (typeof this.node === "string" || typeof this.node=== "number" || typeof this.node === "boolean")
        },
        isArray: function() {
            return Array.isArray(this.node)
        },
        getMargin: function() {
            return "margin-left: " + this.spacing + "px";
        }
    }
}
</script>
