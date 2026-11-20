
import React from 'react';
import { Clock, RefreshCw, CheckCircle2 } from 'lucide-react';
import type { HardwareTelemetry, TransportKind } from '../types/dispenser';
import {
  getAvailableTimezones,
  type TimezoneClockSnapshot,
} from '../utils/timezone';

interface DeviceClockProps {
  telemetry: HardwareTelemetry;
  transport: TransportKind;
  timezone: string;
  onChangeTimezone: (tz: string) => void;
  use24Hour: boolean;
  onToggle24Hour: () => void;
  autoSyncClock: boolean;
  onToggleAutoSyncClock: () => void;
  clockSnapshot: TimezoneClockSnapshot;
  onSyncClock: () => Promise<void>;
  isSyncingClock: boolean;
}

export const DeviceClock: React.FC<DeviceClockProps> = ({
  telemetry,
  transport,
  timezone,
  onChangeTimezone,
  use24Hour,
  onToggle24Hour,
  autoSyncClock,
  onToggleAutoSyncClock,
  clockSnapshot,
  onSyncClock,
  isSyncingClock,
}) => {
  const timezones = getAvailableTimezones();

  return (
    <div className="bg-white border border-slate-200 rounded-xl p-6 flex flex-col justify-between h-full">
      <div className="space-y-4">
        <div className="flex items-start justify-between gap-3">
          <div>
            <h2 className="text-lg font-bold text-slate-900 tracking-tight">
              Device Clock
            </h2>
          </div>
          <button
            type="button"
            onClick={onToggle24Hour}
            className="px-2.5 py-1.5 text-xs font-semibold text-slate-700 bg-slate-100 hover:bg-slate-200 rounded-md transition-colors whitespace-nowrap shrink-0 cursor-pointer"
          >
            {use24Hour ? '24h Format' : '12h Format'}
          </button>
        </div>

        <div className="py-4 border-y border-slate-100">
          <div className="font-mono tabular-nums text-3xl sm:text-4xl font-semibold text-slate-900 tracking-tight">
            {use24Hour ? clockSnapshot.time24 : clockSnapshot.time12}
          </div>
          <div className="mt-1.5 flex flex-wrap items-center gap-2 text-xs text-slate-500">
            <span>{clockSnapshot.dateLabel}</span>
            <span aria-hidden="true">·</span>
            <span className="font-mono tabular-nums">{clockSnapshot.offsetLabel}</span>
            <span aria-hidden="true">·</span>
            <span>{clockSnapshot.shortZone}</span>
          </div>
        </div>

        <div>
          <label
            htmlFor="device-timezone-select"
            className="block text-xs font-semibold text-slate-700 mb-1.5"
          >
            Dispenser Timezone
          </label>
          <select
            id="device-timezone-select"
            value={timezone}
            onChange={(e) => onChangeTimezone(e.target.value)}
            className="w-full h-10 px-3 text-sm bg-white border border-slate-300 rounded-lg text-slate-900 focus:outline-none focus:ring-2 focus:ring-sky-600 focus:border-sky-600"
          >
            {timezones.map((tz) => (
              <option key={tz.id} value={tz.id}>
                {tz.label} — {tz.id}
              </option>
            ))}
          </select>
        </div>

        <label className="flex items-center gap-2.5 text-xs text-slate-600 cursor-pointer select-none">
          <input
            type="checkbox"
            checked={autoSyncClock}
            onChange={onToggleAutoSyncClock}
            disabled={transport === 'ble'}
            className="w-4 h-4 rounded border-slate-300 text-sky-700 focus:ring-sky-600"
          />
          <span>
            Automatically sync ESP32 clock when connected or timezone changes (USB Serial)
          </span>
        </label>
      </div>

      <div className="pt-4 mt-4 border-t border-slate-100 flex items-center justify-between gap-3">
        <div className="text-xs text-slate-500 min-w-0">
          {telemetry.lastClockSyncAt ? (
            <span className="inline-flex items-center gap-1.5 text-emerald-700">
              <CheckCircle2 className="w-3.5 h-3.5 shrink-0" aria-hidden="true" />
              <span className="truncate">
                Synced ({telemetry.lastClockSyncTz})
              </span>
            </span>
          ) : (
            <span className="inline-flex items-center gap-1.5">
              <Clock className="w-3.5 h-3.5 shrink-0" aria-hidden="true" />
              <span>Not synced to ESP32 over USB Serial yet</span>
            </span>
          )}
        </div>

        <button
          type="button"
          onClick={() => void onSyncClock()}
          disabled={transport !== 'serial' || isSyncingClock}
          className="inline-flex items-center gap-2 px-4 py-2 text-sm font-semibold text-white bg-slate-900 hover:bg-slate-800 rounded-lg transition-colors disabled:opacity-40 disabled:cursor-not-allowed whitespace-nowrap shrink-0 cursor-pointer"
        >
          <RefreshCw
            className={`w-3.5 h-3.5 ${isSyncingClock ? 'animate-spin' : ''}`}
            aria-hidden="true"
          />
          <span>{isSyncingClock ? 'Syncing...' : 'Sync Clock'}</span>
        </button>
      </div>
    </div>
  );
};
