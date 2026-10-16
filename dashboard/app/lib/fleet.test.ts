import { describe, expect, it } from "vitest";
import { createFleetClient, FleetError, FleetUnavailable } from "./fleet";

type Call = { method: string; path: string; auth?: string; body?: string };
type Reply = { status: number; body?: unknown } | Error;

/** A fetch stand-in that answers each "METHOD /path" with the next queued reply. */
function fakeFetch(replies: Record<string, Reply[]>) {
  const calls: Call[] = [];
  const fetchFn = async (url: string | URL, init?: RequestInit) => {
    const u = new URL(String(url));
    const method = init?.method ?? "GET";
    const path = u.pathname.replace("/api/v1", "");
    const headers = (init?.headers ?? {}) as Record<string, string>;
    calls.push({ method, path, auth: headers.Authorization, body: init?.body as string });
    const queue = replies[`${method} ${path}`];
    if (!queue?.length) throw new Error(`unexpected ${method} ${path}`);
    const reply = queue.shift()!;
    if (reply instanceof Error) throw reply;
    return new Response(reply.body === undefined ? null : JSON.stringify(reply.body), {
      status: reply.status,
    });
  };
  return { fetchFn: fetchFn as typeof fetch, calls };
}

const tokens = (n: number) => ({
  status: 200,
  body: { access_token: `access-${n}`, refresh_token: `refresh-${n}`, token_type: "bearer" },
});

function client(fetchFn: typeof fetch) {
  return createFleetClient({
    baseUrl: "http://fleet.test/api/v1",
    user: "dash@example.com",
    password: "pw",
    fetchFn,
  });
}

describe("fleet client", () => {
  it("logs in with a form post and sends the access token", async () => {
    const { fetchFn, calls } = fakeFetch({
      "POST /auth/login": [tokens(1)],
      "GET /robots": [{ status: 200, body: { items: [] } }],
    });
    await client(fetchFn).request("GET", "/robots");
    expect(calls[0].body).toBe("username=dash%40example.com&password=pw");
    expect(calls[1].auth).toBe("Bearer access-1");
  });

  it("refreshes once on 401 and retries with the new token", async () => {
    const { fetchFn, calls } = fakeFetch({
      "POST /auth/login": [tokens(1)],
      "GET /robots": [{ status: 401 }, { status: 200, body: { items: [] } }],
      "POST /auth/refresh": [tokens(2)],
    });
    await client(fetchFn).request("GET", "/robots");
    expect(calls.map((c) => `${c.method} ${c.path}`)).toEqual([
      "POST /auth/login", "GET /robots", "POST /auth/refresh", "GET /robots",
    ]);
    expect(JSON.parse(calls[2].body!)).toEqual({ refresh_token: "refresh-1" });
    expect(calls[3].auth).toBe("Bearer access-2");
  });

  it("logs in again when the refresh is refused, and does not loop", async () => {
    const { fetchFn, calls } = fakeFetch({
      "POST /auth/login": [tokens(1), tokens(2)],
      "GET /robots": [{ status: 401 }, { status: 401 }],
      "POST /auth/refresh": [{ status: 401 }],
    });
    await expect(client(fetchFn).request("GET", "/robots")).rejects.toBeInstanceOf(FleetError);
    expect(calls).toHaveLength(5);
    expect(calls[4].auth).toBe("Bearer access-2");
  });

  it("turns a network failure into FleetUnavailable", async () => {
    const { fetchFn } = fakeFetch({ "POST /auth/login": [new TypeError("fetch failed")] });
    await expect(client(fetchFn).request("GET", "/robots")).rejects.toBeInstanceOf(
      FleetUnavailable
    );
  });

  it("reports the API's status and detail on other errors", async () => {
    const { fetchFn } = fakeFetch({
      "POST /auth/login": [tokens(1)],
      "POST /robots/9/goals": [{ status: 404, body: { detail: "Robot not found" } }],
    });
    const err = await client(fetchFn)
      .request("POST", "/robots/9/goals", { json: { x: 0, y: 0 } })
      .catch((e) => e);
    expect(err).toBeInstanceOf(FleetError);
    expect(err.status).toBe(404);
    expect(err.message).toContain("Robot not found");
  });

  it("retries once when a reused connection was already closed by the API", async () => {
    // uvicorn closes idle keep-alive connections after 5 s; a request that
    // lands on one fails with ECONNRESET before the API reads it (Gate 7).
    const reset = Object.assign(new TypeError("fetch failed"), { cause: { code: "ECONNRESET" } });
    const { fetchFn, calls } = fakeFetch({
      "POST /auth/login": [tokens(1)],
      "POST /robots/1/goals": [reset, { status: 201, body: { id: 5 } }],
    });
    const goal = await client(fetchFn).request("POST", "/robots/1/goals", { json: { x: 1, y: 2 } });
    expect(goal).toEqual({ id: 5 });
    expect(calls.filter((c) => c.path === "/robots/1/goals")).toHaveLength(2);
  });

  it("gives up after a second connection reset", async () => {
    const reset = () => Object.assign(new TypeError("fetch failed"), { cause: { code: "ECONNRESET" } });
    const { fetchFn } = fakeFetch({
      "POST /auth/login": [tokens(1)],
      "GET /robots": [reset(), reset()],
    });
    await expect(client(fetchFn).request("GET", "/robots")).rejects.toBeInstanceOf(FleetUnavailable);
  });

  it("does not retry a refused connection", async () => {
    const refused = Object.assign(new TypeError("fetch failed"), { cause: { code: "ECONNREFUSED" } });
    const { fetchFn, calls } = fakeFetch({ "POST /auth/login": [refused] });
    await expect(client(fetchFn).request("GET", "/robots")).rejects.toBeInstanceOf(FleetUnavailable);
    expect(calls).toHaveLength(1);
  });

  it("shares one login between requests made before there is a token", async () => {
    const { fetchFn, calls } = fakeFetch({
      "POST /auth/login": [tokens(1)],
      "GET /robots": [{ status: 200, body: { items: [] } }, { status: 200, body: { items: [] } }],
    });
    const c = client(fetchFn);
    await Promise.all([c.request("GET", "/robots"), c.request("GET", "/robots")]);
    expect(calls.filter((call) => call.path === "/auth/login")).toHaveLength(1);
  });

  it("sends query parameters", async () => {
    const { fetchFn, calls } = fakeFetch({
      "POST /auth/login": [tokens(1)],
      "GET /robots/1/goals": [{ status: 200, body: [] }],
    });
    await client(fetchFn).request("GET", "/robots/1/goals", { params: { status: "pending" } });
    expect(calls[1].path).toBe("/robots/1/goals");
  });
});
