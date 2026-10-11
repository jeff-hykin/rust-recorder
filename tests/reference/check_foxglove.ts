// Decodes every message of a ROS 2 MCAP with Foxglove's own libraries (the ones its app uses), from the definitions
// embedded in the file, and compares with `<file>.expected.json` (from examples/reference_fixture.rs).
// deno run -A tests/reference/check_foxglove.ts <file.mcap>
import { McapStreamReader } from "npm:@mcap/core@2"
import { fixupTypes, parse } from "npm:@foxglove/rosmsg@5"
import { MessageReader } from "npm:@foxglove/rosmsg2-serialization@3"

const path = Deno.args[0]
const expected = JSON.parse(await Deno.readTextFile(path.replace(/\.mcap$/, ".expected.json")))
const reader = new McapStreamReader({ validateCrcs: true })
reader.append(await Deno.readFile(path))

const plain = (value: unknown): unknown => {
    if (ArrayBuffer.isView(value)) {
        return Array.from(value as unknown as ArrayLike<number | bigint>, (item) => typeof item === "bigint" ? Number(item) : item)
    }
    if (Array.isArray(value)) {
        return value.map(plain)
    }
    if (typeof value === "bigint") {
        return Number(value)
    }
    if (value && typeof value === "object") {
        return Object.fromEntries(Object.entries(value).map(([key, item]) => [key, plain(item)]))
    }
    return value
}

const same = (actual: any, wanted: any, path: string) => {
    if (wanted && typeof wanted === "object" && !Array.isArray(wanted)) {
        const keys = Object.keys(wanted).sort().join(",")
        if (Object.keys(actual ?? {}).sort().join(",") !== keys) {
            throw new Error(`${path}: fields ${Object.keys(actual ?? {})} != ${keys}`)
        }
        for (const key of Object.keys(wanted)) {
            same(actual[key], wanted[key], `${path}.${key}`)
        }
    } else if (Array.isArray(wanted)) {
        if (actual.length !== wanted.length) {
            throw new Error(`${path}: ${actual.length} items, expected ${wanted.length}`)
        }
        wanted.forEach((item, index) => same(actual[index], item, `${path}[${index}]`))
    } else if (typeof wanted === "number") {
        // int64 values past 2^53 come back as BigInt → Number, so compare with that precision
        if (Math.abs(Number(actual) - wanted) > Math.abs(wanted) * 1e-12 + 1e-12) {
            throw new Error(`${path}: ${actual} != ${wanted}`)
        }
    } else if (typeof wanted === "boolean") {
        if (Boolean(actual) !== wanted) {
            throw new Error(`${path}: ${actual} != ${wanted}`)
        }
    } else if (actual !== wanted) {
        throw new Error(`${path}: ${JSON.stringify(actual)} != ${JSON.stringify(wanted)}`)
    }
}

const schemas = new Map<number, { name: string; reader: MessageReader }>()
const channels = new Map<number, { topic: string; schemaId: number; messageEncoding: string }>()
const seen = new Map<string, number>()
let profile = ""
for (let record; (record = reader.nextRecord());) {
    if (record.type === "Header") {
        profile = record.profile
    } else if (record.type === "Schema") {
        if (record.encoding !== "ros2msg") {
            throw new Error(`schema ${record.name} is ${record.encoding}`)
        }
        // exactly how Foxglove's ROS 2 decoder loads an MCAP schema
        const definitions = parse(new TextDecoder().decode(record.data), { ros2: true, skipTypeFixup: true })
        fixupTypes(definitions)
        schemas.set(record.id, { name: record.name, reader: new MessageReader(definitions) })
    } else if (record.type === "Channel") {
        channels.set(record.id, record)
    } else if (record.type === "Message") {
        const channel = channels.get(record.channelId)!
        if (channel.messageEncoding !== "cdr") {
            throw new Error(`${channel.topic} is ${channel.messageEncoding}`)
        }
        const schema = schemas.get(channel.schemaId)!
        const index = seen.get(channel.topic) ?? 0
        seen.set(channel.topic, index + 1)
        const want = expected[channel.topic]
        if (schema.name !== want.schema) {
            throw new Error(`${channel.topic}: schema ${schema.name} != ${want.schema}`)
        }
        same(plain(schema.reader.readMessage(record.data)), want.messages[index], `${channel.topic}#${index}`)
    }
}
if (profile !== "ros2") {
    throw new Error(`profile ${profile}`)
}
for (const [topic, want] of Object.entries<any>(expected)) {
    if ((seen.get(topic) ?? 0) !== want.messages.length) {
        throw new Error(`${topic}: read ${seen.get(topic) ?? 0} of ${want.messages.length}`)
    }
}
console.log(`Foxglove's decoders read ${[...seen.values()].reduce((a, b) => a + b, 0)} messages on ${seen.size} topics, all as recorded`)
