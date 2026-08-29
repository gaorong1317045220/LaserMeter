// SPDX-License-Identifier: MIT
// Focused regression test for project reset and scan-history state handling.

const fs = require('fs');
const vm = require('vm');

const html = fs.readFileSync('pc_app/static/index.html', 'utf8');
const source = html.slice(html.indexOf('<script>') + '<script>'.length, html.lastIndexOf('</script>'));
const elements = new Map();

function element(id) {
    if (!elements.has(id)) {
        elements.set(id, {
            id,
            style: {},
            classList: { add() {}, remove() {}, toggle() {} },
            addEventListener() {},
            setAttribute() {},
            removeAttribute() {},
            load() {},
            querySelector() { return null; },
            querySelectorAll() { return []; },
            getBoundingClientRect() { return { width: 1000, height: 600, left: 0, top: 0 }; },
            options: [],
            value: '',
            selectedIndex: 0,
            textContent: '',
            innerHTML: '',
        });
    }
    return elements.get(id);
}

const storage = new Map();
const context = {
    console,
    document: {
        getElementById: element,
        addEventListener() {},
        querySelector() { return null; },
        querySelectorAll() { return []; },
        createElement() { return element('created'); },
        body: { appendChild() {}, removeChild() {} },
    },
    window: { addEventListener() {} },
    localStorage: {
        getItem: key => storage.has(key) ? storage.get(key) : null,
        setItem: (key, value) => storage.set(key, String(value)),
        removeItem: key => storage.delete(key),
    },
    confirm: () => true,
    setTimeout,
    clearTimeout,
    setInterval: () => 0,
    clearInterval,
    URL: { createObjectURL: () => '', revokeObjectURL() {} },
    Blob: function Blob() {},
    fetch: async () => { throw new Error('fetch not configured'); },
    structuredClone: global.structuredClone,
};
vm.createContext(context);

const tests = String.raw`
(async () => {
    updateAll = () => {};
    clearSelection = () => {
        appState.selectedId = null;
        appState.selectedType = null;
        appState.multiSelected = [];
        appState.marquee = null;
    };
    fitCanvas = () => {};
    showToast = () => {};

    appState.walls = [{ id: 'old' }];
    appState.scanPoints = [{ x_mm: 1, y_mm: 2 }];
    appState.scanResult = { path: 'old.csv' };
    scanLoadRequestSerial = 7;
    localStorage.setItem('room-measure-project', 'old');
    localStorage.setItem('room-measure-project-name', 'old');
    app.newProject();
    if (appState.walls.length || appState.scanPoints.length) throw new Error('newProject did not clear canvas state');
    if (appState.scanResult !== null) throw new Error('newProject did not clear scanResult');
    if (scanLoadRequestSerial !== 8) throw new Error('newProject did not invalidate pending scan');
    if (localStorage.getItem('room-measure-project') !== null) throw new Error('newProject left recovery state');
    if (document.getElementById('scan-history').selectedIndex !== -1) throw new Error('newProject left history selected');

    appState.walls = [
        { id: 'wall_old', ax: 0, ay: 0, bx: 1000, by: 0, thickness: 180, height: 2800,
            source: 'scan_ransac', scanSourcePath: 'old.csv' },
        { id: 'wall_manual', ax: 0, ay: 0, bx: 0, by: 1000, thickness: 180, height: 2800 },
    ];
    appState.doors = [{ id: 'door_old', wallId: 'wall_old', offset: 100, width: 800, height: 2100 }];
    appState.windows = [];
    appState.sockets = [];
    appState.scanSourcePath = 'history.csv';
    appState.scanProcessingSummary = { model_ready: true };
    appState.scanFittedWalls = [{ id: 'fit1', ax_mm: 10, ay_mm: 20, bx_mm: 1010, by_mm: 20,
        length_mm: 1000, rms_error_mm: 1, inlier_count: 10 }];
    adoptScanWalls(true);
    if (appState.walls.some(wall => wall.id === 'wall_old')) throw new Error('old scan walls were retained');
    if (!appState.walls.some(wall => wall.id === 'wall_manual')) throw new Error('manual wall was removed');
    if (!appState.walls.some(wall => wall.source === 'scan_ransac' && wall.scanSourcePath === 'history.csv')) {
        throw new Error('history wall not adopted');
    }
    if (appState.doors.some(door => door.wallId === 'wall_old')) throw new Error('dependent old opening was retained');

    const pending = [];
    fetch = url => new Promise(resolve => pending.push({ url, resolve }));
    updateScanPanel = () => {};
    adoptScanWalls = () => {};
    const result = path => ({
        ok: true,
        path,
        points: [{ x_mm: path === 'new.csv' ? 2 : 1, y_mm: 0 }],
        preview_outline: [],
        processed_geometry: { filtered_points: [], removed_outliers: [], walls: [], corners: [], occlusions: [], summary: {} },
        quality: { outline_ready: false },
    });
    const oldPromise = loadScan('old.csv', true);
    const newPromise = loadScan('new.csv', true);
    pending[1].resolve({ ok: true, json: async () => result('new.csv') });
    await newPromise;
    pending[0].resolve({ ok: true, json: async () => result('old.csv') });
    await oldPromise;
    if (appState.scanSourcePath !== 'new.csv') throw new Error('stale scan response replaced latest history selection');

    console.log('WEB_STATE_REGRESSION_OK');
})()
`;

const result = new vm.Script(source + tests, { filename: 'index.state.test.js' }).runInContext(context);
Promise.resolve(result).catch(error => {
    console.error(error);
    process.exitCode = 1;
});
