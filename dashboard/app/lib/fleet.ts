/**
 * Server-side client for the robot-fleet-management API (DECISIONS #3): used
 * only by the dashboard's route handlers, so the tokens never reach the
 * browser. Same auth rules as fleet_bridge's client: on HTTP 401 refresh once
 * and retry; if the refresh is refused, log in again and retry once more.
 */

export class FleetUnavailable extends Error {}

export class FleetError extends Error {
  constructor(public status: number, public detail: unknown) {
    super(`fleet API returned ${status}: ${typeof detail === "string" ? detail : JSON.stringify(detail)}`);
  }
}

type Options = { json?: unknown; params?: Record<string, string | number> };

export type FleetClient = {
  request: <T = any>(method: string, path: string, options?: Options) => Promise<T>;
};

export function createFleetClient(config: {
  baseUrl: string;
  user: string;
  password: string;
  fetchFn?: typeof fetch;
}): FleetClient {
  const fetchFn = config.fetchFn ?? fetch;
  const base = config.baseUrl.replace(/\/$/, "");
  let access: string | null = null;
  let refresh: string | null = null;

  async function send(method: string, path: string, init: { body?: string; type?: string;
    token?: string | null; params?: Options["params"] }) {
    const query = init.params
      ? "?" + new URLSearchParams(Object.entries(init.params).map(([k, v]) => [k, String(v)]))
      : "";
    const headers: Record<string, string> = { Accept: "application/json" };
    if (init.type) headers["Content-Type"] = init.type;
    if (init.token) headers.Authorization = `Bearer ${init.token}`;
    try {
      return await fetchFn(base + path + query, {
        method, headers, body: init.body, cache: "no-store",
      });
    } catch (err) {
      const cause = (err as { cause?: { code?: string } })?.cause?.code ?? String(err);
      throw new FleetUnavailable(`${method} ${path}: no answer from the fleet API (${cause})`);
    }
  }

  async function detail(resp: Response) {
    const text = await resp.text();
    try {
      const body = JSON.parse(text);
      return body?.detail ?? body;
    } catch {
      return text.slice(0, 200);
    }
  }

  async function store(resp: Response) {
    const body = await resp.json();
    access = body.access_token;
    refresh = body.refresh_token;
  }

  async function login() {
    const resp = await send("POST", "/auth/login", {
      body: new URLSearchParams({ username: config.user, password: config.password }).toString(),
      type: "application/x-www-form-urlencoded",
    });
    if (resp.status !== 200) throw new FleetError(resp.status, await detail(resp));
    await store(resp);
  }

  async function tryRefresh() {
    if (!refresh) return false;
    const resp = await send("POST", "/auth/refresh", {
      body: JSON.stringify({ refresh_token: refresh }),
      type: "application/json",
    });
    if (resp.status !== 200) return false;
    await store(resp);
    return true;
  }

  async function request(method: string, path: string, options: Options = {}) {
    if (!access) await login();
    const init = {
      body: options.json === undefined ? undefined : JSON.stringify(options.json),
      type: options.json === undefined ? undefined : "application/json",
      params: options.params,
    };
    let resp = await send(method, path, { ...init, token: access });
    if (resp.status === 401) {
      if (!(await tryRefresh())) await login();
      resp = await send(method, path, { ...init, token: access });
    }
    if (!resp.ok) throw new FleetError(resp.status, await detail(resp));
    return resp.status === 204 ? null : resp.json();
  }

  return { request };
}
