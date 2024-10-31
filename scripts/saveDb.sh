#!/bin/bash

echo "Saving databases"
pg_dump -U user-dev -d academy_db --exclude-table public.universes > /docker-entrypoint-initdb.d/exercises/dump.sql
pg_dump -U user-dev -d academy_db --table public.universes > /universes/dump.sql