import disnake
from disnake.ext import commands
from disnake.ext.commands import has_role
import logging
import os
from datetime import datetime

class BankLogs(commands.Cog):
    def __init__(self, bot):
        self.bot = bot
        self.log_filename = 'bank_logs.log'
        self.setup_logger()

    def setup_logger(self):
        """Настройка логгера для записи действий."""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(self.log_filename),
                logging.StreamHandler()
            ]
        )

    @commands.Cog.listener()
    async def on_slash_command(self, inter):
        """Записывает информацию о выполненных командах."""
        command_name = inter.application_command.name
        user = inter.author
        guild = inter.guild.name if inter.guild else "Личные сообщения"
        logging.info(f"Команда '{command_name}' была вызвана пользователем {user} на сервере '{guild}'.")

    @commands.Cog.listener()
    async def on_slash_command_completion(self, inter):
        """Записывает успешное выполнение команды."""
        command_name = inter.application_command.name
        user = inter.author
        logging.info(f"Команда '{command_name}' успешно выполнена пользователем {user}.")

    @commands.Cog.listener()
    async def on_command_error(self, inter, error):
        """Обрабатывает и логирует ошибки команд."""
        command_name = inter.application_command.name if inter.application_command else "Неизвестная команда"
        user = inter.author
        logging.error(f"Ошибка при выполнении команды '{command_name}' пользователем {user}: {error}")

    @commands.slash_command(description="Показывает последние логи банка.")
    @has_role(1327671090738237512)
    async def view_logs(self, inter, lines: int = 10):
        """Команда для просмотра последних записей логов."""
        if not os.path.exists(self.log_filename):
            await inter.send("Лог-файл отсутствует.")
            return

        try:
            with open(self.log_filename, 'r') as f:
                logs = f.readlines()

            last_logs = ''.join(logs[-lines:])
            if not last_logs.strip():
                last_logs = "Нет записей для отображения."

            embed = disnake.Embed(
                title="Последние логи банка",
                description=f"```{last_logs}```",
                color=disnake.Color.purple()
            )
            await inter.send(embed=embed)

        except IOError as e:
            await inter.send(f"Ошибка при чтении лог-файла: {e}")

    @commands.slash_command(description="Очищает лог-файл.")
    @has_role(1327671090738237512)
    async def clear_logs(self, inter):
        """Команда для очистки лог-файла."""
        try:
            with open(self.log_filename, 'w') as f:
                f.write("")
            logging.info(f"Лог-файл очищен пользователем {inter.author}.")
            await inter.send("Лог-файл успешно очищен.")
        except IOError as e:
            await inter.send(f"Ошибка при очистке лог-файла: {e}")

    @commands.slash_command(description="Устанавливает канал для уведомлений логов.")
    @has_role(1327671090738237512)
    async def set_log_channel(self, inter, channel: disnake.TextChannel):
        """Команда для установки канала, куда будут отправляться логи."""
        self.log_channel_id = channel.id
        logging.info(f"Канал логов установлен: {channel.name} (ID: {channel.id}) пользователем {inter.author}.")
        await inter.send(f"Канал логов успешно установлен на {channel.mention}.")

    async def notify_log_channel(self, message):
        """Отправляет уведомления в указанный канал логов."""
        if hasattr(self, 'log_channel_id'):
            channel = self.bot.get_channel(self.log_channel_id)
            if channel:
                await channel.send(message)

    @commands.Cog.listener()
    async def on_any_event(self, event):
        """Пример логирования произвольного события (можно настроить под себя)."""
        logging.info(f"Произошло событие: {event}")


# Регистрация этого cog
def setup(bot):
    bot.add_cog(BankLogs(bot))
