"""Process-local registry populated from the latched detector mapping topic."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ClassInfo:
    id: int
    name: str
    object: str
    camera: str | None
    multi_instance: bool


class ModelClassRegistry:
    """Validated class-ID registry populated from the static ROS message."""

    def __init__(self, *, schema_version=0, model='', entries=()):
        self.schema_version = int(schema_version)
        self.model = str(model)
        self.entries = tuple(entries)
        self._by_id = {entry.id: entry for entry in self.entries}
        self._by_name = {entry.name: entry for entry in self.entries}

    @classmethod
    def from_message(cls, message):
        arrays = (
            list(message.ids), list(message.names), list(message.objects),
            list(message.cameras), list(message.multi_instance),
        )
        if len({len(values) for values in arrays}) != 1:
            raise ValueError('model class mapping arrays have inconsistent lengths')
        entries = []
        seen_ids = set()
        seen_names = set()
        for values in zip(*arrays):
            class_id, name, object_name, camera, multi_instance = values
            class_id = int(class_id)
            name = str(name).strip().lower().replace('-', '_').replace(' ', '_')
            object_name = (str(object_name or name).strip().lower()
                           .replace('-', '_').replace(' ', '_'))
            camera = str(camera or '').strip().lower() or None
            if camera == 'any':
                camera = None
            if class_id < 0 or class_id in seen_ids or not name or name in seen_names:
                raise ValueError('model class mapping contains duplicate or invalid entries')
            if camera not in (None, 'front', 'down'):
                raise ValueError('model class mapping camera must be front, down, or empty')
            seen_ids.add(class_id)
            seen_names.add(name)
            entries.append(ClassInfo(
                id=class_id, name=name, object=object_name,
                camera=camera, multi_instance=bool(multi_instance)))
        entries.sort(key=lambda entry: entry.id)
        if [entry.id for entry in entries] != list(range(len(entries))):
            raise ValueError('model class mapping IDs must be contiguous from zero')
        return cls(schema_version=message.schema_version,
                   model=message.model, entries=entries)

    @classmethod
    def empty(cls):
        return cls()

    def model_class_id(self, name, required=True):
        if isinstance(name, bool):
            entry = None
        elif isinstance(name, int):
            entry = self._by_id.get(name)
        else:
            entry = self._by_name.get(str(name).strip())
        if entry is None and required:
            available = ', '.join(item.name for item in self.entries)
            raise ValueError(
                'unknown model class {!r}; available classes: {}'.format(
                    name, available))
        return None if entry is None else entry.id

    def class_info(self, class_id):
        """Return mapping metadata for an ID, or None if it is unknown."""
        try:
            return self._by_id.get(int(class_id))
        except (TypeError, ValueError):
            return None

    def model_class_name(self, class_id):
        entry = self._by_id.get(int(class_id))
        return entry.name if entry is not None else 'class_{}'.format(class_id)

    def physical_class_name(self, class_id):
        entry = self._by_id.get(int(class_id))
        return entry.object if entry is not None else self.model_class_name(class_id)

    def camera_hint(self, class_id):
        entry = self._by_id.get(int(class_id))
        return None if entry is None else entry.camera

    def multi_instance_class_ids(self):
        return {entry.id for entry in self.entries if entry.multi_instance}

    def configured_class_id(self, params, parameter_name, class_name,
                            required=True):
        params = params or {}
        value = params.get(parameter_name)
        if value is not None and not (isinstance(value, str) and not value.strip()):
            try:
                class_id = int(value)
            except (TypeError, ValueError) as error:
                raise ValueError(
                    '{} must be an integer class ID, got {!r}'.format(
                        parameter_name, value)) from error
            if self.model_class_id(class_id, required=False) is None:
                raise ValueError(
                    '{}={} is not present in the shared model mapping'.format(
                        parameter_name, class_id))
            return class_id
        return self.model_class_id(class_name, required=required)
